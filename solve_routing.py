"""
Daily delivery routing (VRP) solver.

Reads deliverable orders from Order_Outstanding_Cleaned, assigns them to
3 vehicles using Strict Territorial Partitioning (Zero Cross-Hauls),
enforces 9-hour time limits, and writes the plan to Google Sheets.

Routing Strategy (cluster-first, route-second):
1. CLUSTER: Orders are grouped into compact spatial territories using
   priority-seeded, capacity-balanced k-means. Seeds are anchored on aging
   (expired) POs and far-from-depot stops so the hardest work defines the
   territories rather than being stranded at someone else's edge. A
   distance-minimizing refinement pass then hands boundary stops to the
   neighbouring territory whenever that shortens total driving.
2. ROUTE: Each territory is solved in isolation as a 1-vehicle TSP, which
   guarantees zero cross-hauls between trucks.
3. SLA Protection: H+3 aging POs carry an exponential drop penalty; remote
   stops carry a distance-scaled penalty so they aren't quietly dropped.
4. Smooth Loops: OR-Tools Guided Local Search sequences each territory into
   a clean, non-overlapping loop.
5. Guards: orders larger than a truck, and small geographically detached
   micro-clusters, are backlogged upfront with an explicit reason instead of
   dragging a truck on a long detour.

--------------------------------------------------------------------------
Changes vs. the original version
--------------------------------------------------------------------------
- Single shared depot+orders distance/time matrix is built ONCE per run and
  sliced per zone/scenario, instead of re-requesting OSRM once per zone per
  scenario (was up to 6 network calls for 3 scenarios; now 1).
- Zone solves within a scenario run in parallel (ThreadPoolExecutor) instead
  of sequentially, cutting wall-clock time roughly in proportion to the
  number of vehicles used.
- Zone partitioning balances total BOX volume (not just order count) across
  vehicles, while still respecting strict contiguous north-to-south bands.
- Backlog reason is now inferred (capacity vs. shift-time) instead of always
  reporting "Shift time limit exceeded".
- UTILIZATION_FLAG_THRESHOLD config was defined but never used - vehicle
  summary rows now actually get flagged "Low utilization" / "Unused".
- The "no solution found" path used to return a malformed summary row
  (wrong column count) and silently drop all orders with no reason; fixed.
- Google service-account credentials are now passed in-memory
  (gspread.service_account_from_dict) instead of being written to a
  credentials.json file on disk, which was also never cleaned up if
  authorization failed.
- Sheet reads now retry on transient Google API errors, matching the
  existing retry behavior on writes.
- load_orders validates that required source columns exist and logs how
  many rows were skipped, instead of failing silently/row-by-row.
- OSRM_MAX_POINTS is now actually enforced.
- traceback (imported but unused) is now used to surface a full stack
  trace and a non-zero exit code on unhandled failures, for cron/CI use.
- print() replaced with logging throughout.
--------------------------------------------------------------------------
"""

import os
import sys
import json
import math
import time
import logging
import datetime
import traceback
import concurrent.futures

import requests
import gspread
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ==========================================
# LOGGING
# ==========================================
logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("vrp_router")


# ==========================================
# CONFIG
# ==========================================
def _env_num(name, default, cast=float):
    raw = os.environ.get(name, "")
    if raw is None or str(raw).strip() == "":
        return default
    return cast(raw)


def _env_bool(name, default):
    raw = os.environ.get(name, "")
    if raw is None or str(raw).strip() == "":
        return default
    return str(raw).strip().lower() not in ("0", "false", "no", "")


DEPOT_LAT = _env_num("DEPOT_LAT", -6.238931133669451)
DEPOT_LNG = _env_num("DEPOT_LNG", 106.53540668179242)
DEPOT_NAME = "FF Bitung"

VEHICLES = [
    {"name": "CDE Long 1", "driver": "Ade", "nopol": "B 9391 BXW"},
    {"name": "CDE Long 2", "driver": "Ahmad", "nopol": "B 9389 BXW"},
    {"name": "CDE Long 3", "driver": "Subki", "nopol": "B 9390 BXW"},
]
NUM_VEHICLES = len(VEHICLES)

VEHICLE_CAPACITY_BOX = int(_env_num("VEHICLE_CAPACITY_BOX", 1200))

# Fixed at 9 hours (540 minutes)
SHIFT_LENGTH_MINUTES = int(_env_num("SHIFT_LENGTH_MINUTES", 540))

BREAK_MINUTES = int(_env_num("BREAK_MINUTES", 20))
DEPOT_PREP_MINUTES = int(_env_num("DEPOT_PREP_MINUTES", 10))
EFFECTIVE_SHIFT_MINUTES = max(60, SHIFT_LENGTH_MINUTES - BREAK_MINUTES - DEPOT_PREP_MINUTES)

SHIFT_START_HOUR = 6

SERVICE_TIME_BASE_MINUTES = int(_env_num("SERVICE_TIME_MINUTES", 15))
SERVICE_TIME_FREE_BOX_THRESHOLD = int(_env_num("SERVICE_TIME_FREE_BOX_THRESHOLD", 6))
SERVICE_TIME_PER_EXTRA_BOX_MINUTES = _env_num("SERVICE_TIME_PER_EXTRA_BOX_MINUTES", 1)


def service_time_minutes(box_count):
    if box_count < SERVICE_TIME_FREE_BOX_THRESHOLD:
        return SERVICE_TIME_BASE_MINUTES
    extra_boxes = box_count - (SERVICE_TIME_FREE_BOX_THRESHOLD - 1)
    return int(round(SERVICE_TIME_BASE_MINUTES + SERVICE_TIME_PER_EXTRA_BOX_MINUTES * extra_boxes))


# "auto"     -> DEFAULT. Builds every strategy below, scores each on
#               estimated servable stops and km-per-stop, and uses the best
#               one for the day's actual geography. Clustering is pure
#               arithmetic, so evaluating all of them is nearly free.
# "kmeans"   -> Priority-seeded, workload-balanced spatial k-means: compact,
#               non-overlapping territories. Best when dense pockets are
#               separated east-west (the case latitude banding mangles).
# "sweep"    -> Classic sweep: sort by polar angle around the depot and cut
#               into contiguous wedges radiating outward.
# "latitude" -> Legacy behavior: contiguous north-to-south latitude bands.
CLUSTER_MODE = os.environ.get("CLUSTER_MODE", "auto").strip().lower()

# How far a territory may be loaded beyond one truck's shift before it is
# treated as full (1.15 = may be planned to 115% of the shift, letting the
# solver trim the last few stops rather than leaving trucks idle). Balance
# is by estimated SHIFT MINUTES, not head-count: clamping territories to
# equal stop counts overrides geography and was measured to make routes
# worse. Vehicle box capacity is always a hard cap.
CLUSTER_BALANCE_SLACK = _env_num("CLUSTER_BALANCE_SLACK", 1.15)
CLUSTER_ITERATIONS = int(_env_num("CLUSTER_ITERATIONS", 30, cast=int))
# Post-clustering relocation passes that move boundary stops between clusters
# when doing so shortens the total estimated route length.
CLUSTER_REFINE_PASSES = int(_env_num("CLUSTER_REFINE_PASSES", 4, cast=int))

# Cluster seeding priority. Seeds are placed on aging and far-from-depot
# stops first, so the hardest-to-serve orders anchor a territory instead of
# being stranded at the edge of someone else's.
SEED_AGE_WEIGHT = _env_num("SEED_AGE_WEIGHT", 2.0)
SEED_DISTANCE_WEIGHT = _env_num("SEED_DISTANCE_WEIGHT", 1.0)
# Extra drop-penalty per km from depot, so remote stops aren't quietly
# dropped by the solver just because they're expensive to reach.
FAR_DROP_PENALTY_PER_KM = _env_num("FAR_DROP_PENALTY_PER_KM", 200)
UTILIZATION_FLAG_THRESHOLD = _env_num("UTILIZATION_FLAG_THRESHOLD", 22)
AVG_SPEED_KMH_FALLBACK = 25

PRIORITY_BASE_PENALTY = int(_env_num("PRIORITY_BASE_PENALTY", 10_000))
PRIORITY_EXP_MULTIPLIER = 100

SOLVE_TIME_LIMIT_SECONDS = int(_env_num("SOLVE_TIME_LIMIT_SECONDS", 15, cast=int))

OSRM_URL = os.environ.get("OSRM_URL", "http://localhost:5000/table/v1/driving/")
OSRM_ROUTE_URL = os.environ.get("OSRM_ROUTE_URL", "http://localhost:5000/route/v1/driving/")
OSRM_MAX_POINTS = int(_env_num("OSRM_MAX_POINTS", 5000, cast=int))

OSRM_DEMO_URL = "https://router.project-osrm.org/table/v1/driving/"
OSRM_DEMO_MAX_POINTS = 100

TRAFFIC_TIME_MULTIPLIER = _env_num("TRAFFIC_TIME_MULTIPLIER", 1.5)

# "box"  -> partition contiguous north-to-south bands so each zone carries a
#           roughly equal total box volume (better truck-load balance).
# "count"-> original behavior: split into equal-sized chunks by order count.
ZONE_BALANCE_MODE = os.environ.get("ZONE_BALANCE_MODE", "box").strip().lower()

# Isolation guard: zoning is latitude-band only, so a couple of stops that
# are geographically far from the rest of their band (but happen to fall in
# the same band) get dragged into a truck's route as a costly detour instead
# of being flagged for a human to decide on. 0 = disabled (legacy behavior).
MAX_ISOLATION_KM = _env_num("MAX_ISOLATION_KM", 0)
# A group of stops smaller than this, and smaller than the zone's main
# cluster, that sits more than MAX_ISOLATION_KM from everything else in the
# zone is treated as an isolated micro-cluster rather than routed.
MIN_ISOLATED_CLUSTER_SIZE = int(_env_num("MIN_ISOLATED_CLUSTER_SIZE", 3, cast=int))
# SLA-aged orders bypass the isolation guard - missing an SLA is usually
# worse than an inefficient detour, so these are still attempted regardless.
ISOLATION_PROTECT_AGE_DAYS = int(_env_num("ISOLATION_PROTECT_AGE_DAYS", 3, cast=int))

# Solve each scenario's zones concurrently instead of one after another.
PARALLEL_ZONE_SOLVE = _env_bool("PARALLEL_ZONE_SOLVE", True)

SOURCE_SHEET = "Order_Outstanding_Cleaned"
OUTPUT_SHEET = "Routing_Plan"
BACKLOG_SHEET = "Routing_Backlog"
SCENARIOS_SHEET = "Routing_Scenarios"
GEOMETRY_SHEET = "Routing_Geometry"
VEHICLE_SUMMARY_SHEET = "Routing_VehicleSummary"

REQUIRED_SOURCE_COLUMNS = [
    "Kode Fulfillment", "Nama Penerima", "Alamat Penerima",
    "Latitude Penerima", "Longitude Penerima", "Total Box",
    "Tanggal Fulfillment Masuk FF",
]


# ==========================================
# GOOGLE SHEETS I/O
# ==========================================
def get_spreadsheet():
    raw_service_account = os.environ.get("GOOGLE_SERVICE_ACCOUNT")
    if not raw_service_account:
        logger.error("FATAL: env var 'GOOGLE_SERVICE_ACCOUNT' is missing/empty.")
        sys.exit(1)
    spreadsheet_id = os.environ.get("SPREADSHEET_ID")
    if not spreadsheet_id:
        logger.error("FATAL: env var 'SPREADSHEET_ID' is missing/empty.")
        sys.exit(1)

    try:
        service_account_info = json.loads(raw_service_account)
    except json.JSONDecodeError as e:
        logger.error(f"FATAL: GOOGLE_SERVICE_ACCOUNT is not valid JSON: {e}")
        sys.exit(1)

    scopes = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
    # Credentials are handed to gspread in-memory only - nothing touches disk,
    # so there's nothing to clean up (and nothing to leak) if auth fails.
    gc = gspread.service_account_from_dict(service_account_info, scopes=scopes)
    return gc.open_by_key(spreadsheet_id)


def _safe_col(row, idx, col_name, default=""):
    i = idx.get(col_name)
    if i is None or i >= len(row):
        return default
    return row[i]


def load_orders(spreadsheet):
    sheet = spreadsheet.worksheet(SOURCE_SHEET)
    values = sheet.get_all_values()
    if len(values) < 2:
        return []

    header, *data_rows = values
    idx = {name: i for i, name in enumerate(header)}

    missing = [c for c in REQUIRED_SOURCE_COLUMNS if c not in idx]
    if missing:
        raise RuntimeError(
            f"Source sheet '{SOURCE_SHEET}' is missing required column(s): {missing}"
        )

    orders = []
    skipped = 0
    for row in data_rows:
        try:
            lat = float(row[idx["Latitude Penerima"]])
            lng = float(row[idx["Longitude Penerima"]])
            box = max(0.0, float(row[idx["Total Box"]] or 0))
            if lat == 0 or lng == 0:
                skipped += 1
                continue

            orders.append({
                "kode_fulfillment": row[idx["Kode Fulfillment"]],
                "nama_penerima": row[idx["Nama Penerima"]],
                "alamat_penerima": row[idx["Alamat Penerima"]],
                "lat": lat, "lng": lng, "box": box,
                "tanggal_masuk": row[idx["Tanggal Fulfillment Masuk FF"]],
                "nomor_awb": _safe_col(row, idx, "Nomor AWB"),
                "kabupaten_kota": _safe_col(row, idx, "Kabupaten/Kota Penerima"),
                "jam_masuk": _safe_col(row, idx, "Jam Fulfillment Masuk FF"),
                "jumlah_bku": _safe_col(row, idx, "Jumlah BKU", box),
            })
        except (ValueError, IndexError):
            skipped += 1
            continue

    if skipped:
        logger.warning(
            f"[{SOURCE_SHEET}] skipped {skipped} row(s) with missing/invalid lat, lng, or box values."
        )
    return orders


def parse_date_flexible(value):
    if not value:
        return None
    for fmt in ("%m/%d/%Y", "%d/%m/%Y", "%Y-%m-%d", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.datetime.strptime(str(value).strip(), fmt).date()
        except ValueError:
            continue
    return None


def write_routing_output(spreadsheet, route_date, scenarios, total_orders=0, computed_at=""):
    comparison_rows = []

    for n, result in list(scenarios.items()):
        label = f"{n}Truck" if n == 1 else f"{n}Trucks"
        plan_sheet = f"{OUTPUT_SHEET}_{label}"
        backlog_sheet = f"{BACKLOG_SHEET}_{label}"
        geometry_sheet = f"{GEOMETRY_SHEET}_{label}"
        vehicle_summary_sheet = f"{VEHICLE_SUMMARY_SHEET}_{label}"

        if result.get("error"):
            comparison_rows.append([route_date, n, 0, 0, 0, 0, 0, 0, f"FAILED: {result['error']}"])
            continue

        _upsert_sheet_by_date(spreadsheet, plan_sheet, [
            "Route Date", "Vehicle", "Driver", "Nopol", "Stop Seq", "Kode Fulfillment",
            "Nama Penerima", "Alamat Penerima", "Kabupaten/Kota Penerima", "Latitude", "Longitude",
            "Box (this stop)", "Cumulative Box", "ETA", "ETD",
            "Distance from Prev (km)", "Tanggal Fulfillment Masuk FF",
            "Nomor AWB", "Jam Fulfillment Masuk FF", "Jumlah BKU",
            "Age (Days)",
        ], [
            [route_date, r["vehicle"], r["driver"], r["nopol"], r["seq"], r["kode_fulfillment"],
             r["nama_penerima"], r["alamat_penerima"], r["kabupaten_kota"], r["lat"], r["lng"],
             r["box"], r["cumulative_box"], r["eta"], r["etd"],
             round(r["distance_from_prev_km"], 2), r["tanggal_masuk"],
             r["nomor_awb"], r["jam_masuk"], r["jumlah_bku"], r.get("age", 0)]
            for r in result["routes"]
        ], route_date)

        _upsert_sheet_by_date(spreadsheet, backlog_sheet, [
            "Route Date", "Kode Fulfillment", "Nama Penerima", "Alamat Penerima",
            "Latitude", "Longitude", "Box", "Tanggal Fulfillment Masuk FF", "Age (Days)", "Reason",
        ], [
            [route_date, b["kode_fulfillment"], b["nama_penerima"], b["alamat_penerima"],
             b["lat"], b["lng"], b["box"], b["tanggal_masuk"], b.get("age", 0), b.get("backlog_reason", "Backlogged")]
            for b in result["backlog"]
        ], route_date)

        geometry_rows = build_geometry_rows(route_date, result["routes"])
        _upsert_sheet_by_date(spreadsheet, geometry_sheet, [
            "Route Date", "Vehicle", "Geometry (JSON [[lat,lng],...])",
        ], geometry_rows, route_date)

        _upsert_sheet_by_date(spreadsheet, vehicle_summary_sheet, [
            "Route Date", "Vehicle", "Driver", "Stops", "Total Box",
            "Total Distance (km)", "Total Time (min)", "Capacity Utilization (%)", "Flag",
        ], [
            [route_date] + row for row in result["summary"]
        ], route_date)

        total_distance = round(sum(row[4] for row in result["summary"]), 2)
        total_stops = sum(row[2] for row in result["summary"])
        total_box = sum(row[3] for row in result["summary"])
        vehicles_used = sum(1 for row in result["summary"] if row[2] > 0)

        comparison_rows.append([
            route_date, n, vehicles_used, total_stops, len(result["backlog"]),
            total_box, total_distance,
            round(total_distance / total_stops, 2) if total_stops else 0, plan_sheet,
            total_orders, computed_at, ", ".join(sorted(DISTANCE_SOURCES_USED)) or "n/a",
        ])

    _upsert_sheet_by_date(spreadsheet, SCENARIOS_SHEET, [
        "Route Date", "Vehicles Available", "Vehicles Actually Used", "Stops Served",
        "Stops Backlogged", "Total Box", "Total Distance (km)",
        "Distance per Stop (km)", "See Plan Tab",
        "Total Orders Considered", "Computed At", "Distance Source(s) Used",
    ], comparison_rows, route_date)


def _read_sheet_rows(spreadsheet, sheet_name, max_retries=4):
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
    except gspread.exceptions.WorksheetNotFound:
        return None, []

    for attempt in range(1, max_retries + 1):
        try:
            values = worksheet.get_all_values()
            if not values:
                return None, []
            return values[0], values[1:]
        except gspread.exceptions.APIError as e:
            logger.warning(f"[{sheet_name}] read attempt {attempt}/{max_retries} failed: {e}")
            time.sleep(3 * (2 ** (attempt - 1)))

    logger.error(f"[{sheet_name}] failed to read after {max_retries} attempts; treating as empty.")
    return None, []


def _upsert_sheet_by_date(spreadsheet, sheet_name, header, new_rows, route_date, max_retries=4):
    existing_header, existing_rows = _read_sheet_rows(spreadsheet, sheet_name, max_retries=max_retries)

    if existing_header and existing_header != header:
        logger.warning(f"[{sheet_name}] existing header doesn't match current schema - rebuilding.")
        existing_rows = []

    kept_rows = [row for row in existing_rows if len(row) == 0 or row[0] != route_date]
    combined_rows = kept_rows + new_rows

    return _write_sheet(spreadsheet, sheet_name, header, combined_rows, max_retries=max_retries)


def fetch_route_geometry(depot_lat, depot_lng, stops):
    if not stops:
        return None
    coords = [(depot_lng, depot_lat)] + [(s["lng"], s["lat"]) for s in stops] + [(depot_lng, depot_lat)]
    coord_str = ";".join(f"{lng},{lat}" for lng, lat in coords)
    url = OSRM_ROUTE_URL + coord_str
    try:
        response = requests.get(url, params={"overview": "full", "geometries": "geojson"}, timeout=30)
        response.raise_for_status()
        data = response.json()
        if data.get("code") != "Ok" or not data.get("routes"):
            return None
        coords_geojson = data["routes"][0]["geometry"]["coordinates"]
        return [[c[1], c[0]] for c in coords_geojson]
    except Exception as e:
        logger.warning(f"[geometry] OSRM route call failed: {e}")
        return None


def build_geometry_rows(route_date, routes):
    by_vehicle = {}
    for r in routes:
        by_vehicle.setdefault(r["vehicle"], []).append(r)

    geometry_rows = []
    for vehicle, stops in by_vehicle.items():
        stops_sorted = sorted(stops, key=lambda r: r["seq"])
        geometry = fetch_route_geometry(DEPOT_LAT, DEPOT_LNG, stops_sorted)
        if geometry:
            geometry_rows.append([route_date, vehicle, json.dumps(geometry)])
    return geometry_rows


def _write_sheet(spreadsheet, sheet_name, header, rows, max_retries=4):
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
    except gspread.exceptions.WorksheetNotFound:
        worksheet = spreadsheet.add_worksheet(title=sheet_name, rows="200", cols=str(max(len(header), 1)))

    for attempt in range(1, max_retries + 1):
        try:
            worksheet.clear()
            if rows or header:
                worksheet.update(range_name="A1", values=[header] + rows)
            logger.info(f"[{sheet_name}] wrote {len(rows)} row(s).")
            return True
        except gspread.exceptions.APIError as e:
            logger.warning(f"[{sheet_name}] write attempt {attempt}/{max_retries} failed: {e}")
            time.sleep(3 * (2 ** (attempt - 1)))

    logger.error(f"[{sheet_name}] failed to write after {max_retries} attempts.")
    return False


# ==========================================
# ROUTING & DISTANCE
# ==========================================
def haversine_km(lat1, lng1, lat2, lng2):
    R = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lng2 - lng1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def build_haversine_matrices(locations):
    n = len(locations)
    dist_km, time_min = [[0.0] * n for _ in range(n)], [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                km = haversine_km(locations[i]["lat"], locations[i]["lng"], locations[j]["lat"], locations[j]["lng"])
                dist_km[i][j] = km
                time_min[i][j] = (km / AVG_SPEED_KMH_FALLBACK) * 60.0
    return dist_km, time_min


def build_osrm_local_matrices(locations):
    coords = ";".join(f"{loc['lng']},{loc['lat']}" for loc in locations)
    url = OSRM_URL + coords

    logger.info(f"[OSRM] Requesting {len(locations)}x{len(locations)} matrix from localhost...")
    response = requests.get(url, params={"annotations": "distance,duration"}, timeout=30)
    response.raise_for_status()
    data = response.json()

    distances_m, durations_s = data["distances"], data["durations"]
    n = len(locations)
    dist_km, time_min = [[0.0] * n for _ in range(n)], [[0.0] * n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i != j:
                d, t = distances_m[i][j], durations_s[i][j]
                if d is None or t is None:
                    km = haversine_km(locations[i]["lat"], locations[i]["lng"], locations[j]["lat"], locations[j]["lng"])
                    dist_km[i][j] = km
                    time_min[i][j] = (km / AVG_SPEED_KMH_FALLBACK) * 60.0
                else:
                    dist_km[i][j] = d / 1000.0
                    time_min[i][j] = t / 60.0
    return dist_km, time_min


def build_osrm_demo_matrices(locations):
    if len(locations) > OSRM_DEMO_MAX_POINTS:
        raise RuntimeError(f"{len(locations)} points exceeds demo limit.")

    coords = ";".join(f"{loc['lng']},{loc['lat']}" for loc in locations)
    url = OSRM_DEMO_URL + coords

    logger.info(f"[OSRM-demo] Requesting {len(locations)}x{len(locations)} matrix from public demo...")
    response = requests.get(url, params={"annotations": "distance,duration"}, timeout=30,
                             headers={"User-Agent": "auto-routing-vrp/1.0"})
    response.raise_for_status()
    data = response.json()

    if data.get("code") != "Ok":
        raise RuntimeError(f"OSRM demo server returned: {data.get('code')} - {data.get('message', '')}")

    distances_m, durations_s = data["distances"], data["durations"]
    n = len(locations)
    dist_km, time_min = [[0.0] * n for _ in range(n)], [[0.0] * n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i != j:
                d, t = distances_m[i][j], durations_s[i][j]
                if d is None or t is None:
                    km = haversine_km(locations[i]["lat"], locations[i]["lng"], locations[j]["lat"], locations[j]["lng"])
                    dist_km[i][j] = km
                    time_min[i][j] = (km / AVG_SPEED_KMH_FALLBACK) * 60.0
                else:
                    dist_km[i][j] = d / 1000.0
                    time_min[i][j] = t / 60.0
    return dist_km, time_min


def _apply_traffic_buffer(time_min):
    n = len(time_min)
    return [[time_min[i][j] * TRAFFIC_TIME_MULTIPLIER for j in range(n)] for i in range(n)]


DISTANCE_SOURCES_USED = set()


def build_distance_time_matrices(locations):
    if os.environ.get("SKIP_OSM"):
        DISTANCE_SOURCES_USED.add("Straight-line (SKIP_OSM forced)")
        dist_km, time_min = build_haversine_matrices(locations)
        return dist_km, _apply_traffic_buffer(time_min)

    if len(locations) <= OSRM_MAX_POINTS:
        try:
            dist_km, time_min = build_osrm_local_matrices(locations)
            DISTANCE_SOURCES_USED.add("Local OSRM")
            return dist_km, _apply_traffic_buffer(time_min)
        except Exception as e:
            logger.warning(f"[OSRM] Local routing failed ({e}); trying public demo.")
    else:
        logger.warning(
            f"{len(locations)} points exceeds OSRM_MAX_POINTS ({OSRM_MAX_POINTS}); skipping local OSRM."
        )

    try:
        dist_km, time_min = build_osrm_demo_matrices(locations)
        DISTANCE_SOURCES_USED.add("Public OSRM demo")
        return dist_km, _apply_traffic_buffer(time_min)
    except Exception as e:
        logger.warning(f"[OSRM-demo] Failed ({e}); falling back to straight-line.")

    DISTANCE_SOURCES_USED.add("Straight-line (all OSRM sources failed)")
    dist_km, time_min = build_haversine_matrices(locations)
    return dist_km, _apply_traffic_buffer(time_min)


def build_global_matrices(depot_lat, depot_lng, orders):
    """Build the depot + all-orders distance/time matrix exactly once per run.
    Zone-level matrices for every scenario are sliced out of this instead of
    each triggering their own OSRM request."""
    locations = [{"lat": depot_lat, "lng": depot_lng}] + [
        {"lat": o["lat"], "lng": o["lng"]} for o in orders
    ]
    return build_distance_time_matrices(locations)


def slice_matrix(dist_km_full, time_min_full, global_indices):
    """Pull out the depot + a subset of orders (by their index into the
    original `orders` list) from the full matrix, preserving the order of
    `global_indices` so row/col i+1 matches global_indices[i]."""
    idxs = [0] + [g + 1 for g in global_indices]
    zdist = [[dist_km_full[a][b] for b in idxs] for a in idxs]
    ztime = [[time_min_full[a][b] for b in idxs] for a in idxs]
    return zdist, ztime


def partition_zones_by_count(orders_sorted, n):
    chunk_size = math.ceil(len(orders_sorted) / n)
    zones = []
    for i in range(n):
        chunk = orders_sorted[i * chunk_size: (i + 1) * chunk_size]
        if chunk:
            zones.append(chunk)
    return zones


def partition_zones_by_box(orders_sorted, n):
    """Same contiguous north-to-south ordering as partition_zones_by_count,
    but each cut point is chosen so every zone carries roughly 1/n of the
    total box volume rather than 1/n of the order count. This keeps the
    'strict territorial, zero cross-haul' property while balancing truck
    load much better when order sizes vary a lot."""
    total_box = sum(o.get("box", 0) for o in orders_sorted)
    if total_box <= 0 or n <= 1:
        return partition_zones_by_count(orders_sorted, n)

    target = total_box / n
    zones = [[] for _ in range(n)]
    zone_idx = 0
    running = 0.0
    for o in orders_sorted:
        zones[zone_idx].append(o)
        running += o.get("box", 0)
        if running >= target and zone_idx < n - 1:
            zone_idx += 1
            running = 0.0
    return [z for z in zones if z]


# ==========================================
# CLUSTER-FIRST TERRITORY BUILDING
# ==========================================
def _to_xy(lat, lng, lat0):
    """Equirectangular projection to kilometres. Over a metro-sized area this
    is accurate enough for centroid math and far cheaper than repeated
    haversine calls inside the clustering loops."""
    R = 6371.0
    x = math.radians(lng) * math.cos(math.radians(lat0)) * R
    y = math.radians(lat) * R
    return x, y


def _prepare_geometry(orders, depot_lat, depot_lng):
    """Attach projected coordinates and depot distance to each order."""
    lat0 = depot_lat
    dx, dy = _to_xy(depot_lat, depot_lng, lat0)
    prepared = []
    for o in orders:
        x, y = _to_xy(o["lat"], o["lng"], lat0)
        prepared.append({
            **o,
            "_x": x, "_y": y,
            "_depot_km": math.hypot(x - dx, y - dy),
            "_angle": math.atan2(y - dy, x - dx),
        })
    return prepared, (dx, dy)


def _priority_score(order, max_depot_km):
    """Higher = more important that this stop anchors a territory.
    Expired/aging POs dominate; distance from depot breaks ties so remote
    stops get their own territory instead of being stranded on the edge of
    someone else's."""
    age = min(order.get("age", 0), 4)
    far = (order["_depot_km"] / max_depot_km) if max_depot_km > 0 else 0.0
    return SEED_AGE_WEIGHT * age + SEED_DISTANCE_WEIGHT * far


def _select_seeds(orders, n):
    """Priority-weighted k-means++ seeding. The first seed is the highest
    priority stop (most aged / furthest out), and each subsequent seed is
    chosen far from existing seeds and weighted toward high-priority stops,
    so territories form around the hard-to-serve work rather than around
    whatever happens to be dense."""
    if not orders or n <= 0:
        return []
    max_depot_km = max(o["_depot_km"] for o in orders) or 1.0
    scored = sorted(orders, key=lambda o: -_priority_score(o, max_depot_km))

    seeds = [(scored[0]["_x"], scored[0]["_y"])]
    while len(seeds) < n and len(seeds) < len(orders):
        best, best_val = None, -1.0
        for o in orders:
            d2 = min((o["_x"] - sx) ** 2 + (o["_y"] - sy) ** 2 for sx, sy in seeds)
            weight = 1.0 + _priority_score(o, max_depot_km)
            val = d2 * weight
            if val > best_val:
                best, best_val = o, val
        if best is None:
            break
        seeds.append((best["_x"], best["_y"]))
    return seeds


def _estimate_cluster_minutes(cluster, depot_xy):
    """Estimate how long a territory takes to serve in a shift, without
    running the solver. Travel uses the Beardwood-Halton-Hammersley
    approximation (expected TSP length ~ 0.7124 * sqrt(n * area)) plus the
    out-and-back radial leg. This is what lets a DENSE territory correctly
    take on more stops than a spread-out one: a flat stop-count cap treats
    30 tightly-packed metro drops the same as 30 scattered rural ones, which
    is exactly how trucks end up either overloaded or idle."""
    n = len(cluster)
    if n == 0:
        return 0.0

    service = sum(service_time_minutes(o.get("box", 0)) for o in cluster)

    xs = [o["_x"] for o in cluster]
    ys = [o["_y"] for o in cluster]
    area = max((max(xs) - min(xs)), 0.1) * max((max(ys) - min(ys)), 0.1)
    intra_km = 0.7124 * math.sqrt(max(1, n) * area) if n > 1 else 0.0

    dx, dy = depot_xy
    radial_km = 2.0 * (sum(math.hypot(x - dx, y - dy) for x, y in zip(xs, ys)) / n)

    travel_min = ((intra_km + radial_km) / AVG_SPEED_KMH_FALLBACK) * 60.0 * TRAFFIC_TIME_MULTIPLIER
    return service + travel_min


def _assign_balanced(orders, centroids, max_depot_km, depot_xy):
    """Assign every order to a centroid, respecting per-cluster caps on box
    volume and on estimated shift workload. Orders are placed in order of
    'regret' (how much worse their second-best cluster is) with
    high-priority stops going first, so the stops that would suffer most
    from a bad assignment get their preferred territory.

    The workload cap is what balances the plan: territories fill until they
    run out of SHIFT, not until they hit an arbitrary head-count, so dense
    areas absorb more stops and no truck sits idle while another overflows."""
    n = len(centroids)
    if n == 0:
        return []
    box_cap = VEHICLE_CAPACITY_BOX
    workload_cap = EFFECTIVE_SHIFT_MINUTES * CLUSTER_BALANCE_SLACK

    ranked = []
    for o in orders:
        dists = sorted(
            ((math.hypot(o["_x"] - cx, o["_y"] - cy), ci) for ci, (cx, cy) in enumerate(centroids)),
            key=lambda t: t[0],
        )
        regret = (dists[1][0] - dists[0][0]) if len(dists) > 1 else 0.0
        ranked.append((o, dists, regret))

    ranked.sort(key=lambda t: (-_priority_score(t[0], max_depot_km), -t[2]))

    clusters = [[] for _ in range(n)]
    loads_box = [0.0] * n

    for o, dists, _ in ranked:
        placed = False
        for d, ci in dists:
            if loads_box[ci] + o.get("box", 0) > box_cap:
                continue
            if _estimate_cluster_minutes(clusters[ci] + [o], depot_xy) > workload_cap:
                continue
            clusters[ci].append(o)
            loads_box[ci] += o.get("box", 0)
            placed = True
            break
        if not placed:
            # Every territory is full on shift time or capacity. Put it with
            # the nearest cluster that can physically carry the box load and
            # let the solver backlog the overflow with an explicit reason.
            feasible = [(d, ci) for d, ci in dists if loads_box[ci] + o.get("box", 0) <= box_cap]
            if feasible:
                _, ci = feasible[0]
            else:
                ci = min(range(n), key=lambda c: _estimate_cluster_minutes(clusters[c], depot_xy))
            clusters[ci].append(o)
            loads_box[ci] += o.get("box", 0)

    return clusters


def _centroid(cluster):
    if not cluster:
        return None
    return (
        sum(o["_x"] for o in cluster) / len(cluster),
        sum(o["_y"] for o in cluster) / len(cluster),
    )


def _estimate_route_km(cluster, depot_xy, dist_km_full=None):
    """Cheap nearest-neighbour route length estimate, used to compare
    candidate clusterings without invoking the full solver."""
    if not cluster:
        return 0.0
    dx, dy = depot_xy
    remaining = list(cluster)
    cur = (dx, dy)
    total = 0.0
    while remaining:
        nxt = min(remaining, key=lambda o: math.hypot(o["_x"] - cur[0], o["_y"] - cur[1]))
        total += math.hypot(nxt["_x"] - cur[0], nxt["_y"] - cur[1])
        cur = (nxt["_x"], nxt["_y"])
        remaining.remove(nxt)
    total += math.hypot(dx - cur[0], dy - cur[1])
    return total


def _refine_clusters(clusters, depot_xy, passes):
    """Relocate boundary stops between clusters whenever doing so shortens
    the combined estimated route length. This is the step that removes the
    'long finger' detours - a stop that geographically belongs to a
    neighbouring territory gets handed over instead of being driven to.

    A move is only accepted when it flows from a BUSIER territory to a
    LIGHTER one (measured in estimated shift minutes, not stop count).
    Without that guard the greedy distance objective happily drains a
    territory down to a handful of stops (leaving a truck nearly idle),
    because merging work into one dense loop is always shorter in raw km."""
    workload_cap = EFFECTIVE_SHIFT_MINUTES * CLUSTER_BALANCE_SLACK

    for _ in range(max(0, passes)):
        improved = False
        costs = [_estimate_route_km(c, depot_xy) for c in clusters]
        minutes = [_estimate_cluster_minutes(c, depot_xy) for c in clusters]

        for src in range(len(clusters)):
            if len(clusters[src]) <= 1:
                continue
            for o in list(clusters[src]):
                best_gain, best_dst = 0.0, None
                for dst in range(len(clusters)):
                    # Balance guard: only move work toward a lighter truck.
                    if dst == src or minutes[dst] >= minutes[src]:
                        continue
                    if sum(x.get("box", 0) for x in clusters[dst]) + o.get("box", 0) > VEHICLE_CAPACITY_BOX:
                        continue
                    new_src = [x for x in clusters[src] if x is not o]
                    new_dst = clusters[dst] + [o]
                    if _estimate_cluster_minutes(new_dst, depot_xy) > workload_cap:
                        continue
                    gain = (costs[src] + costs[dst]) - (
                        _estimate_route_km(new_src, depot_xy) + _estimate_route_km(new_dst, depot_xy)
                    )
                    if gain > best_gain:
                        best_gain, best_dst = gain, dst
                if best_dst is not None:
                    clusters[src].remove(o)
                    clusters[best_dst].append(o)
                    costs[src] = _estimate_route_km(clusters[src], depot_xy)
                    costs[best_dst] = _estimate_route_km(clusters[best_dst], depot_xy)
                    minutes[src] = _estimate_cluster_minutes(clusters[src], depot_xy)
                    minutes[best_dst] = _estimate_cluster_minutes(clusters[best_dst], depot_xy)
                    improved = True
        if not improved:
            break
    return clusters


def build_clusters_kmeans(orders, n, depot_lat, depot_lng):
    """Priority-seeded, capacity-balanced spatial k-means, then a
    distance-minimizing refinement pass. Produces compact, non-overlapping
    territories instead of wide latitude bands."""
    prepared, depot_xy = _prepare_geometry(orders, depot_lat, depot_lng)
    if n <= 1:
        return [prepared]

    max_depot_km = max(o["_depot_km"] for o in prepared) or 1.0
    centroids = _select_seeds(prepared, n)
    clusters = [[] for _ in range(n)]

    for _ in range(max(1, CLUSTER_ITERATIONS)):
        clusters = _assign_balanced(prepared, centroids, max_depot_km, depot_xy)
        new_centroids = []
        for ci, c in enumerate(clusters):
            cen = _centroid(c)
            new_centroids.append(cen if cen else centroids[ci])
        shift = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(centroids, new_centroids))
        centroids = new_centroids
        if shift < 0.01:
            break

    clusters = _refine_clusters(clusters, depot_xy, CLUSTER_REFINE_PASSES)
    return [c for c in clusters if c]


def build_clusters_sweep(orders, n, depot_lat, depot_lng):
    """Classic sweep: order stops by polar angle around the depot and cut
    into contiguous wedges balanced by box volume. Wedges never overlap, so
    routes radiate cleanly outward from the depot."""
    prepared, _ = _prepare_geometry(orders, depot_lat, depot_lng)
    if n <= 1:
        return [prepared]

    by_angle = sorted(prepared, key=lambda o: o["_angle"])
    total_box = sum(o.get("box", 0) for o in by_angle)
    if total_box <= 0:
        return partition_zones_by_count(by_angle, n)

    target = total_box / n
    clusters = [[] for _ in range(n)]
    ci, running = 0, 0.0
    for o in by_angle:
        clusters[ci].append(o)
        running += o.get("box", 0)
        if running >= target and ci < n - 1:
            ci += 1
            running = 0.0
    return [c for c in clusters if c]


def _estimate_cluster_minutes_accurate(cluster, depot_xy):
    """Accurate-enough shift estimate using a real nearest-neighbour route
    length instead of the BHH area approximation. BHH assumes points spread
    through a rectangle, so it badly UNDERESTIMATES long thin territories
    (a wide latitude band looks cheap because its bounding box is flat) -
    which made such territories look servable when they aren't. Used for
    scoring candidate clusterings, where it runs once per candidate rather
    than once per insertion."""
    if not cluster:
        return 0.0
    service = sum(service_time_minutes(o.get("box", 0)) for o in cluster)
    route_km = _estimate_route_km(cluster, depot_xy)
    travel_min = (route_km / AVG_SPEED_KMH_FALLBACK) * 60.0 * TRAFFIC_TIME_MULTIPLIER
    return service + travel_min


def score_clustering(clusters, depot_xy):
    """Score a candidate clustering without running the solver. Primary
    objective is servable stops (stops that actually fit inside a shift),
    secondary is distance per servable stop - i.e. the user's stated
    objective: minimal distance, without quietly serving fewer customers."""
    servable = 0
    total_km = 0.0
    for c in clusters:
        if not c:
            continue
        minutes = _estimate_cluster_minutes_accurate(c, depot_xy)
        if minutes <= EFFECTIVE_SHIFT_MINUTES or minutes <= 0:
            servable += len(c)
        else:
            # Proportionally estimate how much of this territory fits.
            servable += max(1, int(len(c) * EFFECTIVE_SHIFT_MINUTES / minutes))
        total_km += _estimate_route_km(c, depot_xy)
    km_per_stop = total_km / servable if servable else float("inf")
    return servable, km_per_stop, total_km


def build_clusters(orders, n, depot_lat, depot_lng):
    """Cluster first, route second. Returns a list of clusters (territories),
    ordered north-to-south by centroid so vehicle assignment stays stable
    run-to-run.

    In "auto" mode every strategy is built and scored, and the best one for
    THIS day's geography wins. Clustering is pure arithmetic (no network, no
    solver), so trying them all costs milliseconds - and no single strategy
    is best on every layout: latitude banding is fine when deliveries stack
    north-to-south, sweep suits work radiating evenly from the depot, and
    k-means handles dense pockets separated east-west (the case that
    produced the interleaved routes)."""
    if not orders:
        return []
    if n <= 1:
        prepared, _ = _prepare_geometry(orders, depot_lat, depot_lng)
        return [prepared]

    def _latitude_clusters():
        srt = sorted(orders, key=lambda x: x["lat"], reverse=True)
        raw = (partition_zones_by_box(srt, n) if ZONE_BALANCE_MODE == "box"
               else partition_zones_by_count(srt, n))
        out = []
        for c in raw:
            prepared, _ = _prepare_geometry(c, depot_lat, depot_lng)
            out.append(prepared)
        return out

    builders = {
        "kmeans": lambda: build_clusters_kmeans(orders, n, depot_lat, depot_lng),
        "sweep": lambda: build_clusters_sweep(orders, n, depot_lat, depot_lng),
        "latitude": _latitude_clusters,
    }

    _, depot_xy = _prepare_geometry(orders, depot_lat, depot_lng)

    if CLUSTER_MODE in builders:
        clusters = builders[CLUSTER_MODE]()
    else:
        best, best_key, best_name = None, None, None
        for name, build in builders.items():
            try:
                cand = [c for c in build() if c]
            except Exception as e:
                logger.warning(f"  [cluster] strategy '{name}' failed: {e}")
                continue
            if not cand:
                continue
            servable, km_per_stop, total_km = score_clustering(cand, depot_xy)
            logger.info(
                f"  [cluster:candidate] {name:<9} sizes={[len(c) for c in cand]} "
                f"est. servable={servable} est. {km_per_stop:.2f}km/stop"
            )
            key = (-servable, km_per_stop)
            if best_key is None or key < best_key:
                best, best_key, best_name = cand, key, name
        if best is None:
            best = build_clusters_kmeans(orders, n, depot_lat, depot_lng)
            best_name = "kmeans"
        logger.info(f"  [cluster] selected strategy: {best_name}")
        clusters = best

    clusters = [c for c in clusters if c]
    # Stable vehicle assignment: northernmost territory -> first vehicle.
    clusters.sort(key=lambda c: -(sum(o["lat"] for o in c) / len(c)))
    return clusters


def cluster_quality_metrics(clusters):
    """Diagnostics that quantify exactly the failure visible on the map:
    'interleaving' is the share of stops whose nearest neighbour sits in a
    different cluster (overlapping territories), and spread is the mean
    distance from a stop to its own cluster's centroid (compactness)."""
    all_pts = [(o, ci) for ci, c in enumerate(clusters) for o in c]
    if len(all_pts) < 2:
        return {"interleaving": 0.0, "mean_spread_km": 0.0, "max_spread_km": 0.0}

    interleaved = 0
    for o, ci in all_pts:
        nearest_ci, nearest_d = None, float("inf")
        for p, cj in all_pts:
            if p is o:
                continue
            d = math.hypot(o["_x"] - p["_x"], o["_y"] - p["_y"])
            if d < nearest_d:
                nearest_d, nearest_ci = d, cj
        if nearest_ci != ci:
            interleaved += 1

    spreads = []
    for c in clusters:
        cen = _centroid(c)
        if cen:
            spreads.extend(math.hypot(o["_x"] - cen[0], o["_y"] - cen[1]) for o in c)

    return {
        "interleaving": interleaved / len(all_pts),
        "mean_spread_km": sum(spreads) / len(spreads) if spreads else 0.0,
        "max_spread_km": max(spreads) if spreads else 0.0,
    }


def _classify_backlog_reason(box_used_before, order_box, capacity):
    if box_used_before + order_box > capacity:
        return "Vehicle capacity exceeded"
    return "Shift time limit exceeded (9 hours)"


def _find_distance_clusters(zone_orders, dist_km_full, max_isolation_km):
    """Single-linkage clustering of a zone's orders using the shared full
    matrix: two orders are in the same cluster if there's a chain of stops
    between them each no more than max_isolation_km apart. Returns a list of
    clusters, each a list of indices into zone_orders."""
    n = len(zone_orders)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        gi = zone_orders[i]["_gidx"] + 1
        for j in range(i + 1, n):
            gj = zone_orders[j]["_gidx"] + 1
            if dist_km_full[gi][gj] <= max_isolation_km:
                union(i, j)

    clusters = {}
    for i in range(n):
        clusters.setdefault(find(i), []).append(i)
    return list(clusters.values())


def split_isolated_orders(zone_orders, dist_km_full, max_isolation_km,
                           min_cluster_size, protect_age_days):
    """Pulls out small, geographically detached groups of stops within a zone
    (e.g. 1-2 orders sitting 30+km from the rest of the zone's stops) so the
    solver doesn't send a truck on a large detour to reach them. Only applies
    when there's a genuinely larger 'main route' elsewhere in the zone to be
    isolated from, and never applies to SLA-aged orders."""
    if max_isolation_km <= 0 or len(zone_orders) < 2:
        return zone_orders, []

    clusters = _find_distance_clusters(zone_orders, dist_km_full, max_isolation_km)
    if len(clusters) <= 1:
        return zone_orders, []

    largest_size = max(len(c) for c in clusters)
    kept, isolated = [], []
    for cluster in clusters:
        is_isolated = len(cluster) < min_cluster_size and len(cluster) < largest_size
        for idx in cluster:
            o = zone_orders[idx]
            if is_isolated and o.get("age", 0) < protect_age_days:
                isolated.append({
                    **o,
                    "backlog_reason": (
                        f"Isolated stop ({len(cluster)} order(s) in a micro-cluster "
                        f"more than {max_isolation_km:g}km from the rest of the zone)"
                    ),
                })
            else:
                kept.append(o)
    return kept, isolated


def _split_unroutable_by_capacity(orders):
    """Orders whose box count alone exceeds a single vehicle's capacity can
    never be routed by any scenario, no matter how zones are balanced. Pulling
    them out up front does two things: it reports them with a correct reason
    immediately (rather than relying on the solver to drop them), and it stops
    a single huge outlier order from skewing box-balanced zone partitioning
    for every other (actually deliverable) order in that zone."""
    routable, unroutable = [], []
    for o in orders:
        if o.get("box", 0) > VEHICLE_CAPACITY_BOX:
            unroutable.append({**o, "backlog_reason": "Vehicle capacity exceeded"})
        else:
            routable.append(o)
    return routable, unroutable


def solve_zone_routing(zone_orders, dist_km, time_min):
    """
    Solves a single isolated geographic zone as a 1-vehicle VRP.
    Guarantees zero cross-hauls between vehicles.
    dist_km / time_min must already be sized (1 + len(zone_orders)) and
    aligned so index 0 = depot, index i+1 = zone_orders[i].
    """
    locations = [{"lat": DEPOT_LAT, "lng": DEPOT_LNG}] + zone_orders
    manager = pywrapcp.RoutingIndexManager(len(locations), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_cb(f, t):
        return int(round(dist_km[manager.IndexToNode(f)][manager.IndexToNode(t)] * 1000))

    def time_cb(f, t):
        i, j = manager.IndexToNode(f), manager.IndexToNode(t)
        service = service_time_minutes(locations[i].get("box", 0)) if i != 0 else 0
        return int(round(time_min[i][j] + service))

    def load_cb(f):
        return int(round(locations[manager.IndexToNode(f)].get("box", 0)))

    def stop_cb(f):
        return 0 if manager.IndexToNode(f) == 0 else 1

    dist_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)

    time_idx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(time_idx, 0, EFFECTIVE_SHIFT_MINUTES, True, "Time")

    routing.AddDimensionWithVehicleCapacity(routing.RegisterUnaryTransitCallback(load_cb), 0, [VEHICLE_CAPACITY_BOX], True, "Capacity")
    routing.AddDimensionWithVehicleCapacity(routing.RegisterUnaryTransitCallback(stop_cb), 0, [len(zone_orders)], True, "StopCount")

    # Prioritize aging POs (H+3 gets massive penalty if dropped) and remote
    # stops (which are expensive to reach and would otherwise be the first
    # thing a distance-minimizing solver quietly drops).
    for idx in range(1, len(locations)):
        node = locations[idx]
        age = node.get("age", 0)
        safe_age = min(age, 4)
        penalty = int(PRIORITY_BASE_PENALTY * (PRIORITY_EXP_MULTIPLIER ** safe_age))
        penalty += int(FAR_DROP_PENALTY_PER_KM * dist_km[0][idx])
        routing.AddDisjunction([manager.NodeToIndex(idx)], penalty)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(SOLVE_TIME_LIMIT_SECONDS)

    solution = routing.SolveWithParameters(params)
    if not solution:
        # Solver couldn't produce anything at all: everything goes to backlog,
        # and the summary row must still match the 6-element shape the
        # caller expects ([seq, box, dist, time, utilization%, flag]).
        backlog = [
            {**o, "backlog_reason": "No solution found for zone"} for o in zone_orders
        ]
        return [], backlog, [0, 0, 0.0, 0, 0.0, "No solution found"]

    idx = routing.Start(0)
    node_order = []
    visited = set()
    while not routing.IsEnd(idx):
        node = manager.IndexToNode(idx)
        if node != 0:
            node_order.append(node)
            visited.add(node - 1)
        idx = solution.Value(routing.NextVar(idx))

    v_route = []
    seq = box = dist = arr_min = 0
    prev = 0
    for node in node_order:
        seq += 1
        order_obj = zone_orders[node - 1]
        box += order_obj.get("box", 0)
        leg_dist = dist_km[prev][node]
        dist += leg_dist

        prev_service_min = service_time_minutes(locations[prev].get("box", 0)) if prev != 0 else 0
        arr_min = arr_min + prev_service_min + time_min[prev][node]
        stop_service_min = service_time_minutes(order_obj.get("box", 0))

        v_route.append({
            "seq": seq,
            "kode_fulfillment": order_obj["kode_fulfillment"],
            "nama_penerima": order_obj["nama_penerima"],
            "alamat_penerima": order_obj["alamat_penerima"],
            "lat": order_obj["lat"], "lng": order_obj["lng"],
            "box": order_obj.get("box", 0), "cumulative_box": box,
            "eta": f"{(SHIFT_START_HOUR * 60 + round(arr_min)) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + round(arr_min)) % 60:02.0f}",
            "etd": f"{(SHIFT_START_HOUR * 60 + round(arr_min) + stop_service_min) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + round(arr_min) + stop_service_min) % 60:02.0f}",
            "distance_from_prev_km": leg_dist, "tanggal_masuk": order_obj["tanggal_masuk"],
            "nomor_awb": order_obj.get("nomor_awb", ""),
            "kabupaten_kota": order_obj.get("kabupaten_kota", ""),
            "jam_masuk": order_obj.get("jam_masuk", ""),
            "jumlah_bku": order_obj.get("jumlah_bku", order_obj.get("box", 0)),
            "age": order_obj.get("age", 0),
        })
        prev = node

    final_service_min = service_time_minutes(locations[prev].get("box", 0)) if prev != 0 else 0
    dist += dist_km[prev][0]
    actual_return_time_min = time_min[prev][0] if prev != 0 else 0
    total_time_min = arr_min + final_service_min + actual_return_time_min

    backlog = [
        {**zone_orders[idx], "backlog_reason": _classify_backlog_reason(box, zone_orders[idx].get("box", 0), VEHICLE_CAPACITY_BOX)}
        for idx in range(len(zone_orders)) if idx not in visited
    ]

    utilization_pct = round(100.0 * box / VEHICLE_CAPACITY_BOX, 1) if VEHICLE_CAPACITY_BOX else 0.0
    if seq == 0:
        flag = "Unused"
    elif utilization_pct < UTILIZATION_FLAG_THRESHOLD:
        flag = "Low utilization"
    else:
        flag = ""

    summary = [seq, box, round(dist, 2), round(total_time_min), utilization_pct, flag]
    return v_route, backlog, summary


def solve_clustered_scenario(orders, n, dist_km_full, time_min_full):
    """
    Strict Territorial Partitioning:
    Splits orders into N geographic zones (North to South) and solves each
    independently, in parallel when PARALLEL_ZONE_SOLVE is enabled.
    """
    if not orders:
        return [], [], []

    routable_orders, unroutable_backlog = _split_unroutable_by_capacity(orders)
    if unroutable_backlog:
        logger.warning(
            f"{len(unroutable_backlog)} order(s) exceed vehicle capacity alone "
            "and can never be routed; backlogged upfront."
        )
    if not routable_orders:
        return [], unroutable_backlog, []

    # STEP 1: CLUSTER (territory building)
    zones = build_clusters(routable_orders, n, DEPOT_LAT, DEPOT_LNG)
    metrics = cluster_quality_metrics(zones)
    logger.info(
        f"  [cluster:{CLUSTER_MODE}] {len(zones)} territory(ies), "
        f"sizes={[len(z) for z in zones]}, "
        f"interleaving={metrics['interleaving']:.1%}, "
        f"mean spread={metrics['mean_spread_km']:.1f}km"
    )

    tasks = []
    for zone_idx, zone_orders in enumerate(zones):
        if zone_idx >= NUM_VEHICLES:
            # Should not happen since zones is built with at most n <=
            # NUM_VEHICLES entries, but fold any overflow safely rather
            # than dropping orders.
            logger.warning("More zones than vehicles available; merging overflow into last vehicle's zone.")
            zones[NUM_VEHICLES - 1].extend(zone_orders)
            continue
        tasks.append((zone_idx, zone_orders, VEHICLES[zone_idx]))

    def _solve_one(zone_orders, vehicle):
        if not zone_orders:
            return vehicle, [], [], [0, 0, 0.0, 0, 0.0, "Unused"]

        routable, isolated_backlog = split_isolated_orders(
            zone_orders, dist_km_full, MAX_ISOLATION_KM,
            MIN_ISOLATED_CLUSTER_SIZE, ISOLATION_PROTECT_AGE_DAYS,
        )
        if isolated_backlog:
            logger.info(
                f"[{vehicle['name']}] held back {len(isolated_backlog)} isolated stop(s) "
                f"from the route (>{MAX_ISOLATION_KM:g}km from the rest of the zone)."
            )
        if not routable:
            return vehicle, [], isolated_backlog, [0, 0, 0.0, 0, 0.0, "Unused"]

        global_indices = [o["_gidx"] for o in routable]
        zdist, ztime = slice_matrix(dist_km_full, time_min_full, global_indices)
        v_route, backlog, summary = solve_zone_routing(routable, zdist, ztime)
        for r in v_route:
            r["vehicle"] = vehicle["name"]
            r["driver"] = vehicle["driver"]
            r["nopol"] = vehicle.get("nopol", "")
        return vehicle, v_route, isolated_backlog + backlog, summary

    results_by_idx = {}
    if PARALLEL_ZONE_SOLVE and len(tasks) > 1:
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(tasks)) as ex:
            future_map = {ex.submit(_solve_one, zo, veh): idx for idx, zo, veh in tasks}
            for fut in concurrent.futures.as_completed(future_map):
                results_by_idx[future_map[fut]] = fut.result()
    else:
        for idx, zo, veh in tasks:
            results_by_idx[idx] = _solve_one(zo, veh)

    all_routes, all_backlog, all_summary = [], [], []
    for idx in sorted(results_by_idx):
        vehicle, v_route, backlog, summary = results_by_idx[idx]
        all_routes.extend(v_route)
        all_backlog.extend(backlog)
        all_summary.append([vehicle["name"], vehicle["driver"]] + summary)

    return all_routes, unroutable_backlog + all_backlog, all_summary


def solve_all_scenarios(orders, dist_km_full, time_min_full):
    scenarios = {}
    for n in range(1, NUM_VEHICLES + 1):
        logger.info(f"--- Scenario: {n} vehicle(s) ---")
        try:
            routes, backlog, summary = solve_clustered_scenario(orders, n, dist_km_full, time_min_full)
            served = sum(row[2] for row in summary)
            dist_total = sum(row[4] for row in summary)
            logger.info(f"  Served {served}/{len(orders)}, backlog {len(backlog)}, dist {dist_total:.1f}km")
            scenarios[n] = {"routes": routes, "backlog": backlog, "summary": summary}
        except Exception as e:
            logger.exception(f"  Scenario {n} failed")
            scenarios[n] = {"error": str(e)}
    return scenarios


def main():
    route_date_str = os.environ.get("ROUTE_DATE", "").strip()
    route_date = (
        datetime.datetime.strptime(route_date_str, "%Y-%m-%d").date()
        if route_date_str else datetime.date.today() + datetime.timedelta(days=1)
    )

    logger.info(f"Routing for date: {route_date.isoformat()}")
    spreadsheet = get_spreadsheet()
    orders = load_orders(spreadsheet)
    if not orders:
        logger.info("No orders to route.")
        return

    for i, o in enumerate(orders):
        o["_gidx"] = i
        order_date = parse_date_flexible(o.get("tanggal_masuk"))
        o["age"] = max(0, (route_date - order_date).days) if order_date else 0

    logger.info(f"Building distance/time matrix for depot + {len(orders)} orders...")
    dist_km_full, time_min_full = build_global_matrices(DEPOT_LAT, DEPOT_LNG, orders)

    scenarios = solve_all_scenarios(orders, dist_km_full, time_min_full)
    computed_at = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    write_routing_output(spreadsheet, route_date.strftime("%d/%m/%Y"), scenarios,
                          total_orders=len(orders), computed_at=computed_at)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        logger.error("Unhandled exception in main():\n" + traceback.format_exc())
        sys.exit(1)
