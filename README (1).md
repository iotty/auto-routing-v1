# Fulfillment + Frex data sync

Pulls two sources on every run and writes each to its own Google Sheet tab:

| Source | Sheet tab |
|---|---|
| Fresh Factory export API (`fulfillment_detail`) | `Fulfillment` |
| Frex TMS shipments (DataTables endpoint) | `Frex` |

## Header behavior

Neither tab uses a hardcoded column list:

- **First run** (tab missing, or its header row is empty): the tab is created/initialized and its header is taken directly from whatever columns the source data has that run.
- **Every run after that**: the header row already in the sheet is treated as fixed. Only source columns whose names match that header are copied in; the sheet's column order and any extra columns you've added manually are preserved.

If you ever want to reset a tab's column layout, just clear its header row (row 1) — the next run will re-populate it from the source.

## Required GitHub secrets

Set these under **Settings → Secrets and variables → Actions**:

- `BEARER_TOKEN` — Fresh Factory export API bearer token
- `GOOGLE_SERVICE_ACCOUNT` — the full JSON key of a Google service account, as a single-line string. This account must have **Editor** access on the target spreadsheet (share the sheet with its `client_email`).
- `TMS_EMAIL` / `TMS_PASSWORD` — Frex TMS login credentials

Optional:
- `SPREADSHEET_ID` — only needed if you want to point at a different spreadsheet than the default one hardcoded in `pipeline.py`.

## Running

- **Scheduled**: the workflow in `.github/workflows/sync-data.yml` runs daily at 23:00 UTC. Adjust the cron line to change timing.
- **Manual**: go to the Actions tab → "Sync Fulfillment & Frex Data" → "Run workflow".
- **Locally**: create a `.env` file with the same four variables above, `pip install -r requirements.txt`, then `python pipeline.py`.

## Notes on GitHub Actions specifically

- Each Actions run starts from a clean container, so the Frex TMS session cookie file (`session_cookies.pkl`) never persists between runs — the pipeline just logs in fresh via `TMS_EMAIL`/`TMS_PASSWORD` every time. This is expected and fine; it only matters for local runs where caching the session saves a login round-trip.
- The two sources are fetched concurrently (they're independent), then written to their sheet tabs one after another to avoid hammering the Sheets API at once.
- A run tab-fails independently — if Frex fails, Fulfillment still gets written (and vice versa). Both statuses are logged to a `Cache_Data` tab, and the job exits non-zero if either failed, so GitHub Actions will flag the run.

## Date windows

- **Fulfillment**: first day of last month → tomorrow (unchanged from the original script).
- **Frex**: today − 7 days → tomorrow (a rolling one-week window).

Adjust `fulfillment_date_window()` or the `timedelta(days=7)` in `fetch_frex()` in `pipeline.py` if you want different windows.
