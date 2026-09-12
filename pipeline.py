"""
Merged data pipeline: pulls two data sources and writes each into its own
Google Sheets tab.

  1. FULFILLMENT  - from the Fresh Factory `exports` API (fulfillment_detail)
  2. FREX         - from the Frex TMS shipments DataTables endpoint

Sheet tabs written:
  - "Fulfillment"
  - "Frex"

Header behaviour (applies to BOTH tabs, no hardcoded column list):
  - If the tab is missing or its header row is empty, it's created and the
    header row is taken as-is from whatever columns the source returns
    (first-run "populate" behaviour).
  - On every later run, the EXISTING header row in the sheet is treated as
    the source of truth. Only columns whose names match that header are
    copied over, in that header's order; unmatched source columns are
    ignored and unmatched header columns are left blank. This keeps the
    sheet's column layout stable even if the source adds/reorders columns.

Run target: GitHub Actions (see .github/workflows/sync-data.yml).
Required secrets / env vars:
  BEARER_TOKEN          - Fresh Factory export API bearer token
  GOOGLE_SERVICE_ACCOUNT- JSON string of a Google service-account key,
                          shared with edit access on the target spreadsheet
  TMS_EMAIL             - Frex TMS login email
  TMS_PASSWORD          - Frex TMS login password
Optional:
  SPREADSHEET_ID        - overrides the default spreadsheet id below
"""

import os
import re
import sys
import json
import time
import pickle
import traceback
import urllib.parse
from pathlib import Path
from datetime import datetime, timedelta, date

import pytz
import requests
import pandas as pd
import gspread
from gspread_dataframe import set_with_dataframe
from oauth2client.service_account import ServiceAccountCredentials
from concurrent.futures import ThreadPoolExecutor

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# ==========================================
# 1. CONFIG / CREDENTIALS
# ==========================================
REQUEST_TIMEOUT = 30
EXPORT_TIMEOUT = 600          # fulfillment_detail export can be slow
EXPORT_MAX_RETRIES = 3

DEFAULT_SPREADSHEET_ID = "1XxA_LgepJhqjoPFx5bZNIp0J57ss_uhqjNk6uxs9cYU"
WAREHOUSE_ID = "7cc6cf47-973c-485c-b605-0e2ae40ec1d6"
TENANT_ID = "7b25e5bd-f47c-42ae-bf10-660e9fdbf78f"
REQUEST_ID = "8d3ec4e2-50bb-47c4-a257-f1cc5e635ad1"
FF_EXPORT_URL = "https://api.freshfactory.id/v1/admin/exports"

TMS_BASE_HOST = "https://tms.frex.co.id"
TMS_LOGIN_URL = f"{TMS_BASE_HOST}/login"
TMS_SHIPMENTS_URL = f"{TMS_BASE_HOST}/en/admin/shipments/shipments"
TMS_COOKIE_JAR_FILE = Path(__file__).parent / "session_cookies.pkl"
TMS_LOGIN_EMAIL_FIELD = "email"
TMS_LOGIN_PASSWORD_FIELD = "password"
TMS_LOGIN_TOKEN_FIELD = "_token"

FULFILLMENT_SHEET_NAME = "Fulfillment"
FREX_SHEET_NAME = "Frex"
CACHE_SHEET_NAME = "Cache_Data"


def load_config():
    """Validate env vars and set up API/Sheets clients. Exits with a clear
    message if anything required is missing or invalid."""
    bearer_token = os.environ.get("BEARER_TOKEN")
    if not bearer_token:
        print("FATAL: env var 'BEARER_TOKEN' is missing/empty.")
        sys.exit(1)

    tms_email = os.environ.get("TMS_EMAIL")
    tms_password = os.environ.get("TMS_PASSWORD")
    if not tms_email or not tms_password:
        print("FATAL: env vars 'TMS_EMAIL' / 'TMS_PASSWORD' are missing/empty.")
        sys.exit(1)

    raw_service_account = os.environ.get("GOOGLE_SERVICE_ACCOUNT")
    if not raw_service_account:
        print("FATAL: env var 'GOOGLE_SERVICE_ACCOUNT' is missing/empty.")
        sys.exit(1)
    try:
        service_account_info = json.loads(raw_service_account)
    except json.JSONDecodeError as e:
        print(f"FATAL: 'GOOGLE_SERVICE_ACCOUNT' is not valid JSON: {e}")
        sys.exit(1)

    creds_path = "credentials.json"
    try:
        with open(creds_path, "w") as f:
            json.dump(service_account_info, f)
        scope = [
            "https://spreadsheets.google.com/feeds",
            "https://www.googleapis.com/auth/drive",
        ]
        creds = ServiceAccountCredentials.from_json_keyfile_name(creds_path, scope)
        gc = gspread.authorize(creds)
    except Exception as e:
        print(f"FATAL: failed to authenticate to Google Sheets: {e}")
        sys.exit(1)
    finally:
        try:
            os.remove(creds_path)
        except OSError:
            pass

    spreadsheet_id = os.environ.get("SPREADSHEET_ID", DEFAULT_SPREADSHEET_ID)
    try:
        spreadsheet = gc.open_by_key(spreadsheet_id)
    except gspread.exceptions.SpreadsheetNotFound:
        print(f"FATAL: spreadsheet '{spreadsheet_id}' not found (check SPREADSHEET_ID).")
        sys.exit(1)
    except PermissionError as e:
        service_account_email = service_account_info.get("client_email", "<unknown - check your JSON key>")
        print(
            f"FATAL: no permission to open spreadsheet '{spreadsheet_id}' (HTTP 403).\n"
            f"Fix checklist:\n"
            f"  1. Open the spreadsheet in your browser, click Share, and add this exact "
            f"email as Editor:\n     {service_account_email}\n"
            f"  2. Double-check SPREADSHEET_ID matches the ID in the sheet's URL "
            f"(the part between /d/ and /edit).\n"
            f"  3. In Google Cloud Console, confirm 'Google Sheets API' and 'Google Drive API' "
            f"are both Enabled on the project this service account belongs to.\n"
            f"Original error: {e}"
        )
        sys.exit(1)
    except gspread.exceptions.APIError as e:
        print(f"FATAL: failed to open spreadsheet (Sheets API error): {e}")
        sys.exit(1)

    return bearer_token, tms_email, tms_password, spreadsheet


BEARER_TOKEN, TMS_EMAIL, TMS_PASSWORD, SPREADSHEET = load_config()

FF_HEADERS = {
    "accept": "application/json",
    "authorization": f"Bearer {BEARER_TOKEN}",
    "user-agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)",
}
FF_SESSION = requests.Session()
FF_SESSION.headers.update(FF_HEADERS)

TMS_HEADERS = {
    "Accept": "application/json, text/javascript, */*; q=0.01",
    "Accept-Language": "en-US,en;q=0.9",
    "Referer": TMS_SHIPMENTS_URL,
    "User-Agent": (
        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
        "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36 Edg/152.0.0.0"
    ),
    "X-Requested-With": "XMLHttpRequest",
}
TMS_SESSION = requests.Session()
TMS_SESSION.headers.update(TMS_HEADERS)

TMS_COLUMNS = [
    {"data": "select", "searchable": "false", "orderable": "false"},
    {"data": "id"},
    {"data": "code"},
    {"data": "order_id"},
    {"data": "status_id", "orderable": "false"},
    {"data": "type", "orderable": "false"},
    {"data": "branch_id", "orderable": "false"},
    {"data": "client_id", "orderable": "false"},
    {"data": "tenant_name", "orderable": "false"},
    {"data": "payment_method_id", "orderable": "false"},
    {"data": "from_state_id", "orderable": "false"},
    {"data": "to_state_id", "orderable": "false"},
    {"data": "shipping_date", "orderable": "false"},
    {"data": "po_expired_date", "orderable": "false"},
    {"data": "action", "searchable": "false", "orderable": "false"},
]


# ==========================================
# 2. FULFILLMENT (Fresh Factory export API)
# ==========================================
def fulfillment_date_window():
    """Rolling window: first day of last month -> tomorrow."""
    today = datetime.now()
    end_date = (today + timedelta(days=1)).strftime("%Y-%m-%d")
    first_day_this_month = today.replace(day=1)
    first_day_last_month = (first_day_this_month - timedelta(days=1)).replace(day=1)
    start_date = first_day_last_month.strftime("%Y-%m-%d")
    return start_date, end_date


def get_fresh_export_token():
    url = f"{FF_EXPORT_URL}/generate_token"
    last_error = None
    for attempt in range(1, 4):
        try:
            response = FF_SESSION.get(url, timeout=REQUEST_TIMEOUT)
        except requests.exceptions.Timeout:
            last_error = "timeout"
        except requests.exceptions.RequestException as e:
            last_error = f"connection error ({e})"
        else:
            if response.status_code == 200:
                try:
                    token = response.json().get("data", {}).get("exportToken")
                except (ValueError, AttributeError):
                    token = None
                if token:
                    return token
                last_error = "HTTP 200 but no exportToken in body"
            else:
                last_error = f"HTTP {response.status_code}"
        if attempt < 3:
            time.sleep(2)
    print(f"WARNING: failed to get export token after 3 attempts. Last error: {last_error}")
    return None


def fetch_fulfillment():
    """Returns (df_or_None, status_or_None). status is only set on failure
    or 'no new data'; None means df is good to write."""
    import io

    start_date, end_date = fulfillment_date_window()
    print(f"[Fulfillment] window: {start_date} to {end_date}")
    params = {
        "type": "fulfillment_detail",
        "warehouse_id": WAREHOUSE_ID,
        "tenant_id": TENANT_ID,
        "start_order_date_in": start_date,
        "end_order_date_in": end_date,
        "page": "1",
        "limit": "15",
    }

    last_status = None
    for attempt in range(1, EXPORT_MAX_RETRIES + 1):
        token = get_fresh_export_token()
        if not token:
            return None, "Failed (Token Expired)"

        req_params = dict(params)
        req_params["token"] = token
        req_params["id"] = REQUEST_ID

        try:
            response = FF_SESSION.get(FF_EXPORT_URL, params=req_params, timeout=EXPORT_TIMEOUT)
        except requests.exceptions.Timeout:
            print(f"[Fulfillment] timeout after {EXPORT_TIMEOUT}s. Not auto-retrying "
                  f"(a retry would restart generation, not resume it).")
            return None, "Failed (Request Timeout)"
        except requests.exceptions.RequestException as e:
            last_status = "Failed (Connection Error)"
            print(f"[Fulfillment] connection error (attempt {attempt}/{EXPORT_MAX_RETRIES}): {e}")
        else:
            if response.status_code != 200:
                last_status = f"Failed (API Error {response.status_code})"
                if response.status_code < 500:
                    print(f"[Fulfillment] HTTP {response.status_code} - not retrying.")
                    return None, last_status
                print(f"[Fulfillment] HTTP {response.status_code} (attempt {attempt}/{EXPORT_MAX_RETRIES}).")
            else:
                try:
                    df = pd.read_excel(io.BytesIO(response.content))
                except Exception as e:
                    print(f"[Fulfillment] failed to parse Excel response: {e}")
                    return None, "Failed (Invalid Excel Response)"
                if df.empty:
                    print("[Fulfillment] no new data for this period.")
                    return None, "Successful (No New Data)"
                return df, None

        if attempt < EXPORT_MAX_RETRIES:
            delay = 5 * attempt
            print(f"[Fulfillment] retrying in {delay}s...")
            time.sleep(delay)

    return None, last_status or "Failed (Unknown Error)"


# ==========================================
# 3. FREX (TMS DataTables endpoint)
# ==========================================
def tms_save_cookies():
    with open(TMS_COOKIE_JAR_FILE, "wb") as f:
        pickle.dump(TMS_SESSION.cookies, f)


def tms_load_cookies() -> bool:
    if not TMS_COOKIE_JAR_FILE.exists():
        return False
    try:
        with open(TMS_COOKIE_JAR_FILE, "rb") as f:
            TMS_SESSION.cookies.update(pickle.load(f))
        return True
    except Exception:
        return False


def tms_login():
    print("[Frex] logging in fresh...")
    resp = TMS_SESSION.get(TMS_LOGIN_URL, timeout=30)
    resp.raise_for_status()

    token = None
    m = re.search(
        rf'name=["\']{TMS_LOGIN_TOKEN_FIELD}["\']\s+value=["\']([^"\']+)["\']',
        resp.text,
    )
    if m:
        token = m.group(1)

    payload = {TMS_LOGIN_EMAIL_FIELD: TMS_EMAIL, TMS_LOGIN_PASSWORD_FIELD: TMS_PASSWORD}
    if token:
        payload[TMS_LOGIN_TOKEN_FIELD] = token

    login_headers = {}
    if "XSRF-TOKEN" in TMS_SESSION.cookies:
        login_headers["X-XSRF-TOKEN"] = urllib.parse.unquote(TMS_SESSION.cookies["XSRF-TOKEN"])

    resp = TMS_SESSION.post(TMS_LOGIN_URL, data=payload, headers=login_headers, timeout=30)
    resp.raise_for_status()

    if "fresh_express_session" not in TMS_SESSION.cookies and resp.url == TMS_LOGIN_URL:
        raise RuntimeError(
            "Login did not appear to succeed (still on login page). Check "
            "TMS_EMAIL/TMS_PASSWORD and the LOGIN_*_FIELD names."
        )
    tms_save_cookies()
    print("[Frex] login successful.")


def tms_ensure_authenticated():
    if tms_load_cookies():
        print("[Frex] reusing saved session (if any survived from a previous step/cache).")
    else:
        tms_login()


def tms_build_params(start, length, draw, date_start, date_end):
    params = {
        "draw": draw,
        "order[0][column]": 1,
        "order[0][dir]": "desc",
        "start": start,
        "length": length,
        "search[value]": "",
        "filter[created_at_start]": date_start,
        "filter[created_at_end]": date_end,
    }
    for i, col in enumerate(TMS_COLUMNS):
        params[f"columns[{i}][data]"] = col["data"]
        if "searchable" in col:
            params[f"columns[{i}][searchable]"] = col["searchable"]
        if "orderable" in col:
            params[f"columns[{i}][orderable]"] = col["orderable"]
    return params


def tms_fetch_range(date_start, date_end, page_size=100, max_retries=3,
                     retry_delay=2.0, allow_relogin=True):
    rows = []
    start = 0
    draw = 1
    total = None

    while total is None or start < total:
        params = tms_build_params(start, page_size, draw, date_start, date_end)
        payload = None
        last_error = None

        for attempt in range(1, max_retries + 1):
            try:
                resp = TMS_SESSION.get(TMS_SHIPMENTS_URL, params=params, timeout=30)
            except requests.exceptions.Timeout:
                last_error = "Request timed out."
            except requests.exceptions.ConnectionError:
                last_error = "Could not connect."
            except requests.exceptions.RequestException as e:
                last_error = f"Network error: {e}"
            else:
                if resp.status_code in (401, 403, 419):
                    if allow_relogin:
                        print(f"[Frex] session rejected (HTTP {resp.status_code}) - re-login and retry.")
                        tms_login()
                        return tms_fetch_range(date_start, date_end, page_size,
                                                max_retries, retry_delay, allow_relogin=False)
                    raise RuntimeError(f"Auth failed again (HTTP {resp.status_code}) after fresh login.")
                if resp.status_code == 429:
                    last_error = "Rate limited (HTTP 429)."
                elif resp.status_code >= 500:
                    last_error = f"Server error (HTTP {resp.status_code})."
                elif not resp.ok:
                    raise RuntimeError(f"HTTP {resp.status_code}: {resp.text[:300]}")
                else:
                    content_type = resp.headers.get("Content-Type", "")
                    if "application/json" not in content_type:
                        if allow_relogin:
                            print("[Frex] non-JSON response (likely login redirect) - re-login and retry.")
                            tms_login()
                            return tms_fetch_range(date_start, date_end, page_size,
                                                    max_retries, retry_delay, allow_relogin=False)
                        raise RuntimeError(f"Still non-JSON after fresh login ('{content_type}').")
                    try:
                        payload = resp.json()
                    except ValueError:
                        last_error = "Response was not valid JSON."
                    else:
                        break

            if attempt < max_retries:
                print(f"[Frex] attempt {attempt}/{max_retries} failed ({last_error}). Retrying in {retry_delay:.0f}s...")
                time.sleep(retry_delay)

        if payload is None:
            raise RuntimeError(
                f"Giving up after {max_retries} attempts fetching rows {start}-{start + page_size} "
                f"({date_start} to {date_end}). Last error: {last_error}"
            )
        if "data" not in payload:
            raise RuntimeError(f"Unexpected response shape (no 'data' key). Keys: {list(payload.keys())}")

        if total is None:
            total = payload.get("recordsTotal", 0)
            print(f"[Frex] total records reported: {total}")
            if total == 0:
                break

        data = payload["data"]
        if not data:
            print(f"[Frex] warning: no rows at offset {start} but total={total}. Stopping early.")
            break

        rows.extend(data)
        print(f"[Frex] fetched {len(rows)} / {total}")
        start += page_size
        draw += 1

    if total is not None and len(rows) != total:
        print(f"[Frex] WARNING: fetched {len(rows)} rows but server reported {total}. Data may be incomplete.")

    return rows


def fetch_frex():
    """Returns (df_or_None, status_or_None), mirroring fetch_fulfillment()."""
    try:
        tms_ensure_authenticated()
        today = date.today()
        date_start = (today - timedelta(days=7)).isoformat()
        date_end = (today + timedelta(days=1)).isoformat()
        print(f"[Frex] window: {date_start} to {date_end}")
        rows = tms_fetch_range(date_start, date_end)
    except Exception as e:
        print("[Frex] failed:")
        print(traceback.format_exc())
        return None, f"Failed ({e})"

    if not rows:
        print("[Frex] no rows returned for this window.")
        return None, "Successful (No New Data)"

    df = pd.DataFrame(rows)
    return df, None


# ==========================================
# 4. SHEETS WRITE (shared, dynamic header matching)
# ==========================================
def sanitize_dataframe_for_sheets(df):
    df = df.copy()
    for col in df.columns:
        if pd.api.types.is_datetime64_any_dtype(df[col]):
            df[col] = df[col].apply(
                lambda x: "" if pd.isna(x) else (
                    x.strftime("%Y-%m-%d") if x.time() == datetime.min.time()
                    else x.strftime("%Y-%m-%d %H:%M:%S")
                )
            )
        else:
            df[col] = df[col].replace([float("inf"), float("-inf")], pd.NA)
    df = df.fillna("")
    df = df.astype(str)
    df = df.replace(["nan", "NaT", "None", "<NA>"], "")
    return df


def write_with_retry(worksheet, df_to_write, max_retries=4, base_delay=3):
    last_error = None
    for attempt in range(1, max_retries + 1):
        try:
            worksheet.clear()
            set_with_dataframe(worksheet, df_to_write, resize=True)
            return True
        except gspread.exceptions.APIError as e:
            last_error = e
            status_code = getattr(getattr(e, "response", None), "status_code", None)
            if status_code in (500, 502, 503, 429):
                delay = base_delay * (2 ** (attempt - 1))
                print(f"  retry {attempt}/{max_retries} after HTTP {status_code}, waiting {delay}s...")
                time.sleep(delay)
                continue
            raise
    raise last_error


def update_cache_log(category_name, status_message):
    try:
        try:
            cache_sheet = SPREADSHEET.worksheet(CACHE_SHEET_NAME)
        except gspread.exceptions.WorksheetNotFound:
            cache_sheet = SPREADSHEET.add_worksheet(title=CACHE_SHEET_NAME, rows="100", cols="3")
            cache_sheet.append_row(["Category", "Last Updated (WIB)", "Last Run Status"])

        wib_tz = pytz.timezone("Asia/Jakarta")
        current_time_wib = datetime.now(wib_tz).strftime("%Y-%m-%d %H:%M:%S")

        try:
            cell = cache_sheet.find(category_name, in_column=1)
        except gspread.exceptions.CellNotFound:
            cell = None

        if cell is not None:
            cache_sheet.update(f"B{cell.row}:C{cell.row}", [[current_time_wib, status_message]])
        else:
            cache_sheet.append_row([category_name, current_time_wib, status_message])
    except Exception:
        print(f"WARNING: failed to update {CACHE_SHEET_NAME} for '{category_name}':")
        print(traceback.format_exc())


def write_to_sheet(tab_name, df_fresh):
    """
    Dynamic header matching, no hardcoded column list:
      - tab missing or header row empty -> create/initialize using the
        source DataFrame's own columns as the header (first-run populate).
      - tab has a header -> only copy columns whose names match; order
        follows the sheet's existing header.
    """
    try:
        try:
            worksheet = SPREADSHEET.worksheet(tab_name)
        except gspread.exceptions.WorksheetNotFound:
            print(f"[{tab_name}] tab not found - creating it.")
            worksheet = SPREADSHEET.add_worksheet(title=tab_name, rows="100", cols=str(max(len(df_fresh.columns), 1)))

        existing_header = [h for h in worksheet.row_values(1) if h != ""]

        if existing_header:
            matched_cols = [c for c in existing_header if c in df_fresh.columns]
            unmatched_cols = [c for c in existing_header if c not in df_fresh.columns]
            if unmatched_cols:
                print(f"[{tab_name}] header columns with no match in source (left blank): {unmatched_cols}")
            print(f"[{tab_name}] matched columns: {len(matched_cols)}/{len(existing_header)}")
            df_to_write = df_fresh.reindex(columns=existing_header)
        else:
            print(f"[{tab_name}] empty header - initializing from source columns.")
            df_to_write = df_fresh

        df_to_write = sanitize_dataframe_for_sheets(df_to_write)

        try:
            write_with_retry(worksheet, df_to_write)
        except gspread.exceptions.APIError as e:
            print(f"[{tab_name}] failed to write after retries: {e}")
            update_cache_log(tab_name, "Failed (Sheets API Error After Retries)")
            return False

        print(f"[{tab_name}] updated: {len(df_to_write)} rows written, header preserved.")
        update_cache_log(tab_name, "Successful")
        return True

    except Exception:
        print(f"[{tab_name}] unexpected error while writing:")
        print(traceback.format_exc())
        update_cache_log(tab_name, "Failed (Unexpected Error)")
        return False


# ==========================================
# 5. PIPELINE
# ==========================================
def main():
    job_status = {}

    with ThreadPoolExecutor(max_workers=2) as executor:
        fulfillment_future = executor.submit(fetch_fulfillment)
        frex_future = executor.submit(fetch_frex)
        fulfillment_df, fulfillment_status = fulfillment_future.result()
        frex_df, frex_status = frex_future.result()

    if fulfillment_status is not None:
        update_cache_log(FULFILLMENT_SHEET_NAME, fulfillment_status)
        job_status[FULFILLMENT_SHEET_NAME] = fulfillment_status.startswith("Successful")
    else:
        job_status[FULFILLMENT_SHEET_NAME] = write_to_sheet(FULFILLMENT_SHEET_NAME, fulfillment_df)

    if frex_status is not None:
        update_cache_log(FREX_SHEET_NAME, frex_status)
        job_status[FREX_SHEET_NAME] = frex_status.startswith("Successful")
    else:
        job_status[FREX_SHEET_NAME] = write_to_sheet(FREX_SHEET_NAME, frex_df)

    print("=" * 50)
    print("SUMMARY:")
    any_failed = False
    for tab_name, ok in job_status.items():
        print(f"  {'OK' if ok else 'FAILED'}: {tab_name}")
        if not ok:
            any_failed = True
    print("=" * 50)

    sys.exit(1 if any_failed else 0)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        print("FATAL: unexpected error stopped the pipeline:")
        print(traceback.format_exc())
        sys.exit(1)
