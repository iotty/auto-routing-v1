"""
Merged data pipeline: pulls two data sources and writes each into its own
Google Sheets tab.

  1. SOURCE A (WMS) - from the Warehouse Management export API
  2. SOURCE B (TMS) - from the Transport Management shipments DataTables endpoint

Sheet tabs written (configurable via env vars):
  - Default: "Fulfillment" (or set via WMS_SHEET_NAME)
  - Default: "Frex" (or set via TMS_SHEET_NAME)

Header behaviour (applies to BOTH tabs, no hardcoded column list):
  - If the tab is missing or its header row is empty, it's created and the
    header row is taken as-is from whatever columns the source returns
    (first-run "populate" behaviour).
  - On every later run, the EXISTING header row in the sheet is treated as
    the source of truth. Only columns whose names match that header are
    copied over, in that header's order; unmatched source columns are
    ignored and unmatched header columns are left blank. This keeps the
    sheet's column layout stable even if the source adds/reorders columns.

Run target: CI/CD Runner (e.g., GitHub Actions).
Required secrets / env vars:
  WMS_BEARER_TOKEN       - WMS export API bearer token (or BEARER_TOKEN)
  WMS_EXPORT_URL         - Full base URL for the WMS export endpoint
  WMS_WAREHOUSE_ID       - Target warehouse UUID
  WMS_TENANT_ID          - Target tenant UUID
  WMS_REQUEST_ID         - Request identifier UUID
  TMS_BASE_HOST          - Base URL for the TMS portal
  TMS_EMAIL              - TMS login email
  TMS_PASSWORD           - TMS login password
  GOOGLE_SERVICE_ACCOUNT - JSON string of a Google service-account key
  SPREADSHEET_ID         - Target Google Spreadsheet ID

Optional env vars:
  WMS_SHEET_NAME         - Custom tab name for WMS data (default: "Fulfillment")
  TMS_SHEET_NAME         - Custom tab name for TMS data (default: "Frex")
  CACHE_SHEET_NAME       - Custom tab name for logs (default: "Cache_Data")
  LOCAL_TIMEZONE         - Timezone for logs (default: "Asia/Jakarta")
"""

import io
import os
import re
import sys
import json
import time
import random
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
EXPORT_TIMEOUT = 600          # Export generation can be slow
EXPORT_MAX_RETRIES = 3
ROLLING_WINDOW_DAYS = int(os.environ.get("ROLLING_WINDOW_DAYS", 5))

WMS_SHEET_NAME = os.environ.get("WMS_SHEET_NAME", "").strip().strip('"').strip("'") or "Fulfillment"
TMS_SHEET_NAME = os.environ.get("TMS_SHEET_NAME", "").strip().strip('"').strip("'") or "Frex"
CACHE_SHEET_NAME = os.environ.get("CACHE_SHEET_NAME", "").strip().strip('"').strip("'") or "Cache_Data"
LOCAL_TIMEZONE = os.environ.get("LOCAL_TIMEZONE", "").strip().strip('"').strip("'") or "Asia/Jakarta"

TMS_COOKIE_JAR_FILE = Path(__file__).parent / "session_cookies.pkl"
TMS_LOGIN_EMAIL_FIELD = "email"
TMS_LOGIN_PASSWORD_FIELD = "password"
TMS_LOGIN_TOKEN_FIELD = "_token"


def clean_env(val: str) -> str:
    """Strips accidental whitespace, quotes, and Markdown link wrappers from env vars."""
    if not val:
        return ""
    val = val.strip()
    # If pasted as a Markdown link [label](value), extract the value
    md_match = re.search(r"\[([^\]]+)\]\(([^)]+)\)", val)
    if md_match:
        val = md_match.group(2).strip()
    return val.strip('"').strip("'").strip()


def get_required_env(var_name: str) -> str:
    val = clean_env(os.environ.get(var_name, ""))
    if not val:
        print(f"FATAL: Required environment variable '{var_name}' is missing or empty.")
        sys.exit(1)
    return val


def sanitize_url(var_name: str) -> str:
    """Extracts a clean http(s) URL even if pasted as a Markdown link [url](url)."""
    raw = os.environ.get(var_name, "").strip()
    if not raw:
        print(f"FATAL: Required environment variable '{var_name}' is missing or empty.")
        sys.exit(1)
    # Extract the first clean http:// or https:// URL, stopping at any Markdown/bracket/quote characters
    match = re.search(r"https?://[^\s\]\)\[\"'<>]+", raw)
    if match:
        return match.group(0).rstrip("/")
    cleaned = raw.strip("[]()\"' \t\r\n").rstrip("/")
    return f"https://{cleaned.lstrip('/')}"


def load_config():
    """Validate env vars and set up API/Sheets clients. Exits with a clear
    message if anything required is missing or invalid."""
    wms_token = (os.environ.get("WMS_BEARER_TOKEN") or os.environ.get("BEARER_TOKEN") or "").strip()
    if not wms_token:
        print("FATAL: env var 'WMS_BEARER_TOKEN' / 'BEARER_TOKEN' is missing/empty.")
        sys.exit(1)

    wms_cfg = {
        "token": wms_token,
        "export_url": sanitize_url("WMS_EXPORT_URL"),
        "warehouse_id": get_required_env("WMS_WAREHOUSE_ID"),
        "tenant_id": get_required_env("WMS_TENANT_ID"),
        "request_id": get_required_env("WMS_REQUEST_ID"),
    }

    tms_base = sanitize_url("TMS_BASE_HOST")
    tms_login_env = os.environ.get("TMS_LOGIN_URL", "").strip()
    tms_shipments_env = os.environ.get("TMS_SHIPMENTS_URL", "").strip()

    tms_cfg = {
        "base_host": tms_base,
        "login_url": sanitize_url("TMS_LOGIN_URL") if tms_login_env else f"{tms_base}/en/admin/login",
        "shipments_url": sanitize_url("TMS_SHIPMENTS_URL") if tms_shipments_env else f"{tms_base}/en/admin/shipments/shipments",
        "email": get_required_env("TMS_EMAIL"),
        "password": os.environ.get("TMS_PASSWORD", "").strip(),
    }
    if not tms_cfg["password"]:
        print("FATAL: Required environment variable 'TMS_PASSWORD' is missing or empty.")
        sys.exit(1)

    raw_service_account = os.environ.get("GOOGLE_SERVICE_ACCOUNT", "").strip()
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

    spreadsheet_id = get_required_env("SPREADSHEET_ID")
    try:
        spreadsheet = gc.open_by_key(spreadsheet_id)
    except gspread.exceptions.SpreadsheetNotFound:
        print("FATAL: target spreadsheet not found (check SPREADSHEET_ID).")
        sys.exit(1)
    except PermissionError as e:
        service_account_email = service_account_info.get("client_email", "<unknown - check your JSON key>")
        print(
            f"FATAL: no permission to open spreadsheet (HTTP 403).\n"
            f"Fix checklist:\n"
            f"  1. Share the spreadsheet with Editor access to: {service_account_email}\n"
            f"  2. Double-check SPREADSHEET_ID matches the ID in the sheet's URL.\n"
            f"  3. Confirm 'Google Sheets API' and 'Google Drive API' are enabled.\n"
            f"Original error: {e}"
        )
        sys.exit(1)
    except gspread.exceptions.APIError as e:
        print(f"FATAL: failed to open spreadsheet (Sheets API error): {e}")
        sys.exit(1)

    return wms_cfg, tms_cfg, spreadsheet


WMS_CFG, TMS_CFG, SPREADSHEET = load_config()

WMS_HEADERS = {
    "accept": "application/json",
    "authorization": f"Bearer {WMS_CFG['token']}",
    "user-agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)",
}
WMS_SESSION = requests.Session()
WMS_SESSION.headers.update(WMS_HEADERS)

TMS_USER_AGENT = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/128.0.0.0 Safari/537.36"
)
TMS_SEC_CH_UA = '"Chromium";v="128", "Not;A=Brand";v="24", "Google Chrome";v="128"'

TMS_HEADERS = {
    "User-Agent": TMS_USER_AGENT,
    "Accept-Language": "en-US,en;q=0.9",
    "Accept-Encoding": "gzip, deflate, br",
    "sec-ch-ua": TMS_SEC_CH_UA,
    "sec-ch-ua-mobile": "?0",
    "sec-ch-ua-platform": '"Windows"',
    "Connection": "keep-alive",
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
# 2. SOURCE A: WMS (Export API)
# ==========================================
def get_rolling_date_window():
    """Rolling window: today - ROLLING_WINDOW_DAYS -> today."""
    today = datetime.now()
    start_date = (today - timedelta(days=ROLLING_WINDOW_DAYS)).strftime("%Y-%m-%d")
    end_date = today.strftime("%Y-%m-%d")
    return start_date, end_date


def get_wms_export_token():
    url = f"{WMS_CFG['export_url']}/generate_token"
    last_error = None
    for attempt in range(1, 4):
        try:
            response = WMS_SESSION.get(url, timeout=REQUEST_TIMEOUT)
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
    print(f"[WMS] WARNING: failed to get export token after 3 attempts. Last error: {last_error}")
    return None


def fetch_wms_data():
    """Returns (df_or_None, status_or_None). status is only set on failure
    or 'no new data'; None means df is good to write."""
    start_date, end_date = get_rolling_date_window()
    print(f"[WMS] window: {start_date} to {end_date}")
    params = {
        "type": os.environ.get("WMS_EXPORT_TYPE", "fulfillment_detail"),
        "warehouse_id": WMS_CFG["warehouse_id"],
        "tenant_id": WMS_CFG["tenant_id"],
        "start_order_date_in": start_date,
        "end_order_date_in": end_date,
        "page": "1",
        "limit": "15",
    }

    last_status = None
    for attempt in range(1, EXPORT_MAX_RETRIES + 1):
        token = get_wms_export_token()
        if not token:
            return None, "Failed (Token Expired)"

        req_params = dict(params)
        req_params["token"] = token
        req_params["id"] = WMS_CFG["request_id"]

        try:
            response = WMS_SESSION.get(WMS_CFG["export_url"], params=req_params, timeout=EXPORT_TIMEOUT)
        except requests.exceptions.Timeout:
            print(f"[WMS] timeout after {EXPORT_TIMEOUT}s. Not auto-retrying "
                  f"(a retry would restart generation, not resume it).")
            return None, "Failed (Request Timeout)"
        except requests.exceptions.RequestException as e:
            last_status = "Failed (Connection Error)"
            print(f"[WMS] connection error (attempt {attempt}/{EXPORT_MAX_RETRIES}): {e}")
        else:
            if response.status_code != 200:
                last_status = f"Failed (API Error {response.status_code})"
                if response.status_code < 500:
                    print(f"[WMS] HTTP {response.status_code} - not retrying.")
                    return None, last_status
                print(f"[WMS] HTTP {response.status_code} (attempt {attempt}/{EXPORT_MAX_RETRIES}).")
            else:
                try:
                    df = pd.read_excel(io.BytesIO(response.content))
                except Exception as e:
                    print(f"[WMS] failed to parse Excel response: {e}")
                    return None, "Failed (Invalid Excel Response)"
                if df.empty:
                    print("[WMS] no new data for this period.")
                    return None, "Successful (No New Data)"
                return df, None

        if attempt < EXPORT_MAX_RETRIES:
            delay = 5 * attempt
            print(f"[WMS] retrying in {delay}s...")
            time.sleep(delay)

    return None, last_status or "Failed (Unknown Error)"


# ==========================================
# 3. SOURCE B: TMS (DataTables Endpoint)
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
    base_host = TMS_CFG["base_host"]
    login_url = TMS_CFG["login_url"]

    nav_headers = {
        "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
        "Sec-Fetch-Dest": "document",
        "Sec-Fetch-Mode": "navigate",
        "Sec-Fetch-Site": "none",
        "Sec-Fetch-User": "?1",
        "Upgrade-Insecure-Requests": "1",
    }

    print("[TMS] visiting portal homepage ...")
    TMS_SESSION.get(base_host, headers=nav_headers, timeout=30)
    time.sleep(random.uniform(0.6, 1.4))

    print("[TMS] authenticating session ...")
    resp = TMS_SESSION.get(
        login_url,
        headers={**nav_headers, "Sec-Fetch-Site": "same-origin", "Referer": base_host + "/"},
        timeout=30,
    )
    if resp.status_code == 404:
        raise RuntimeError("TMS login page not found (404). Check TMS_LOGIN_URL / TMS_BASE_HOST.")
    resp.raise_for_status()

    token = None
    m = re.search(
        rf'name=["\']{TMS_LOGIN_TOKEN_FIELD}["\']\s+value=["\']([^"\']+)["\']',
        resp.text,
    )
    if m:
        token = m.group(1)

    payload = {
        TMS_LOGIN_EMAIL_FIELD: TMS_CFG["email"],
        TMS_LOGIN_PASSWORD_FIELD: TMS_CFG["password"],
    }
    if token:
        payload[TMS_LOGIN_TOKEN_FIELD] = token

    time.sleep(random.uniform(0.8, 2.0))

    login_headers = {
        "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
        "Content-Type": "application/x-www-form-urlencoded",
        "Origin": base_host,
        "Referer": login_url,
        "Sec-Fetch-Dest": "document",
        "Sec-Fetch-Mode": "navigate",
        "Sec-Fetch-Site": "same-origin",
        "Sec-Fetch-User": "?1",
        "Upgrade-Insecure-Requests": "1",
    }
    if "XSRF-TOKEN" in TMS_SESSION.cookies:
        login_headers["X-XSRF-TOKEN"] = urllib.parse.unquote(TMS_SESSION.cookies["XSRF-TOKEN"])

    resp = TMS_SESSION.post(login_url, data=payload, headers=login_headers, timeout=30)
    resp.raise_for_status()

    if resp.url.rstrip("/") == login_url.rstrip("/"):
        raise RuntimeError(
            "Login did not appear to succeed (still on login page). Check "
            "TMS_EMAIL/TMS_PASSWORD and form field names."
        )
    tms_save_cookies()
    print("[TMS] login successful.")


def tms_ensure_authenticated():
    if tms_load_cookies():
        print("[TMS] reusing saved session cookies.")
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
    shipments_url = TMS_CFG["shipments_url"]

    if allow_relogin:
        TMS_SESSION.get(
            shipments_url,
            headers={
                "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
                "Referer": TMS_CFG["base_host"] + "/",
                "Sec-Fetch-Dest": "document",
                "Sec-Fetch-Mode": "navigate",
                "Sec-Fetch-Site": "same-origin",
                "Sec-Fetch-User": "?1",
                "Upgrade-Insecure-Requests": "1",
            },
            timeout=30,
        )
        time.sleep(random.uniform(0.4, 1.0))

    xhr_headers = {
        "Accept": "application/json, text/javascript, */*; q=0.01",
        "Referer": shipments_url,
        "Sec-Fetch-Dest": "empty",
        "Sec-Fetch-Mode": "cors",
        "Sec-Fetch-Site": "same-origin",
        "X-Requested-With": "XMLHttpRequest",
    }

    while total is None or start < total:
        params = tms_build_params(start, page_size, draw, date_start, date_end)
        payload = None
        last_error = None

        for attempt in range(1, max_retries + 1):
            try:
                resp = TMS_SESSION.get(shipments_url, params=params, headers=xhr_headers, timeout=30)
            except requests.exceptions.Timeout:
                last_error = "Request timed out."
            except requests.exceptions.ConnectionError:
                last_error = "Could not connect."
            except requests.exceptions.RequestException as e:
                last_error = f"Network error: {e}"
            else:
                if resp.status_code in (401, 403, 419):
                    if allow_relogin:
                        print(f"[TMS] session rejected (HTTP {resp.status_code}) - re-authenticating.")
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
                            print("[TMS] non-JSON response (likely login redirect) - re-authenticating.")
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
                print(f"[TMS] attempt {attempt}/{max_retries} failed ({last_error}). Retrying in {retry_delay:.0f}s...")
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
            print(f"[TMS] total records reported: {total}")
            if total == 0:
                break

        data = payload["data"]
        if not data:
            print(f"[TMS] warning: no rows at offset {start} but total={total}. Stopping early.")
            break

        rows.extend(data)
        print(f"[TMS] fetched {len(rows)} / {total}")
        start += page_size
        draw += 1
        if total is not None and start < total:
            time.sleep(random.uniform(0.3, 0.8))

    if total is not None and len(rows) != total:
        print(f"[TMS] WARNING: fetched {len(rows)} rows but server reported {total}. Data may be incomplete.")

    return rows


def fetch_tms_data():
    """Returns (df_or_None, status_or_None), mirroring fetch_wms_data()."""
    try:
        tms_ensure_authenticated()
        today = date.today()
        date_start = (today - timedelta(days=ROLLING_WINDOW_DAYS)).isoformat()
        date_end = today.isoformat()
        print(f"[TMS] window: {date_start} to {date_end}")
        rows = tms_fetch_range(date_start, date_end)
    except Exception as e:
        print("[TMS] failed:")
        print(traceback.format_exc())
        return None, f"Failed ({e})"

    if not rows:
        print("[TMS] no rows returned for this window.")
        return None, "Successful (No New Data)"

    df = pd.DataFrame(rows)

    if "id" in df.columns:
        before = len(df)
        df = df.drop_duplicates(subset="id", keep="last")
        if len(df) != before:
            print(f"[TMS] dropped {before - len(df)} duplicate row(s) by 'id'.")

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
            cache_sheet.append_row(["Category", "Last Updated", "Last Run Status"])

        tz = pytz.timezone(LOCAL_TIMEZONE)
        current_time_str = datetime.now(tz).strftime("%Y-%m-%d %H:%M:%S")

        try:
            cell = cache_sheet.find(category_name, in_column=1)
        except gspread.exceptions.CellNotFound:
            cell = None

        if cell is not None:
            cache_sheet.update(
                range_name=f"B{cell.row}:C{cell.row}",
                values=[[current_time_str, status_message]],
            )
        else:
            cache_sheet.append_row([category_name, current_time_str, status_message])
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
        wms_future = executor.submit(fetch_wms_data)
        tms_future = executor.submit(fetch_tms_data)
        wms_df, wms_status = wms_future.result()
        tms_df, tms_status = tms_future.result()

    if wms_status is not None:
        update_cache_log(WMS_SHEET_NAME, wms_status)
        job_status[WMS_SHEET_NAME] = wms_status.startswith("Successful")
    else:
        job_status[WMS_SHEET_NAME] = write_to_sheet(WMS_SHEET_NAME, wms_df)

    if tms_status is not None:
        update_cache_log(TMS_SHEET_NAME, tms_status)
        job_status[TMS_SHEET_NAME] = tms_status.startswith("Successful")
    else:
        job_status[TMS_SHEET_NAME] = write_to_sheet(TMS_SHEET_NAME, tms_df)

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
