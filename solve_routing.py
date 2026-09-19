"""
Daily delivery routing (VRP) solver.

Reads deliverable orders from Order_Outstanding_Cleaned, assigns them to
3 vehicles subject to capacity/time constraints, and writes the
plan to Google Sheets. Backlogs older orders last.
"""

import os
import sys
import json
import math
import time
import datetime
import traceback
import requests
import gspread
from oauth2client.service_account import ServiceAccountCredentials
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ==========================================
# CONFIG (Now dynamic via Apps Script / GitHub Actions)
# ==========================================
def _env_num(name, default, cast=float):
    """Reads an env var as a number, treating an empty string the same as
    "not set" - falls back to default instead of crashing. This matters
    because a workflow_dispatch input left blank when triggered manually
    from the GitHub UI is passed through as an empty string, not omitted
    entirely, and int('') / float('') both raise."""
    raw = os.environ.get(name, "")
    if raw is None or str(raw).strip() == "":
        return default
    return cast(raw)


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
SHIFT_LENGTH_MINUTES = int(_env_num("SHIFT_LENGTH_MINUTES", 600))
# Service time per stop is piecewise: a flat base for small orders (setup/
# paperwork time dominates when there's not much to unload), then scaling
# up per box once an order is big enough that unload time actually starts
# to matter. Mirrors real dispatch software's variable-duration modeling
# better than either a pure flat time or a pure linear-from-zero formula.
SERVICE_TIME_BASE_MINUTES = int(_env_num("SERVICE_TIME_MINUTES", 15))
SERVICE_TIME_FREE_BOX_THRESHOLD = int(_env_num("SERVICE_TIME_FREE_BOX_THRESHOLD", 6))
SERVICE_TIME_PER_EXTRA_BOX_MINUTES = _env_num("SERVICE_TIME_PER_EXTRA_BOX_MINUTES", 1)


def service_time_minutes(box_count):
    """Total service (dwell) time at a stop, given its box count. Rounded
    to a whole minute the SAME way everywhere it's used (the solver's own
    time-budget accounting AND the displayed ETD), so the shift-length
    constraint the solver actually enforces and what a dispatcher sees on
    screen never silently drift apart.

    < SERVICE_TIME_FREE_BOX_THRESHOLD boxes: flat SERVICE_TIME_BASE_MINUTES.
    >= threshold: base + (box_count - (threshold - 1)) extra minutes, i.e.
    the boxes AT and ABOVE the threshold are what add time, not the whole
    count from zero. Example with defaults (base=15, threshold=6, rate=1):
    5 boxes -> 15 min, 6 boxes -> 16 min, 10 boxes -> 20 min.
    """
    if box_count < SERVICE_TIME_FREE_BOX_THRESHOLD:
        return SERVICE_TIME_BASE_MINUTES
    extra_boxes = box_count - (SERVICE_TIME_FREE_BOX_THRESHOLD - 1)
    return int(round(SERVICE_TIME_BASE_MINUTES + SERVICE_TIME_PER_EXTRA_BOX_MINUTES * extra_boxes))

UTILIZATION_FLAG_THRESHOLD = 22
SHIFT_START_HOUR = 6
AVG_SPEED_KMH_FALLBACK = 25

# Base penalty prevents dropping new orders to save distance.
# Exponential multiplier ensures older orders always outrank newer ones.
# A brand-new order's penalty for being backlogged. Distance costs are in
# the same units (km * 1000), so this is directly comparable to a detour
# distance: e.g. 15,000 means "worth serving unless it costs more than
# ~15km of extra driving to reach." Lower than the previous default
# (10,000,000, a 500x bias against ever backlogging even a brand-new
# order) - that extreme value was set to fix an earlier problem (orders
# dropped to save a few km) but over-corrected into never letting the
# solver weigh "is this one isolated order worth a 20-30km detour today,
# or better served once more orders accumulate in that area." The
# exponential escalation below still makes an order effectively mandatory
# within a day or two, so this doesn't reintroduce indefinite deferral.
PRIORITY_BASE_PENALTY = int(_env_num("PRIORITY_BASE_PENALTY", 15_000))
PRIORITY_EXP_MULTIPLIER = 100

SOLVE_TIME_LIMIT_SECONDS = 15

# Pointing to the local OSRM server running in GitHub Actions
OSRM_URL = os.environ.get("OSRM_URL", "http://localhost:5000/table/v1/driving/")
OSRM_ROUTE_URL = os.environ.get("OSRM_ROUTE_URL", "http://localhost:5000/route/v1/driving/")
OSRM_MAX_POINTS = 5000

SOURCE_SHEET = "Order_Outstanding_Cleaned"
OUTPUT_SHEET = "Routing_Plan"
BACKLOG_SHEET = "Routing_Backlog"
SCENARIOS_SHEET = "Routing_Scenarios"
GEOMETRY_SHEET = "Routing_Geometry"
VEHICLE_SUMMARY_SHEET = "Routing_VehicleSummary"

# ==========================================
# GOOGLE SHEETS I/O
# ==========================================
def get_spreadsheet():
    raw_service_account = os.environ.get("GOOGLE_SERVICE_ACCOUNT")
    if not raw_service_account:
        print("FATAL: env var 'GOOGLE_SERVICE_ACCOUNT' is missing/empty.")
        sys.exit(1)
    spreadsheet_id = os.environ.get("SPREADSHEET_ID")
    if not spreadsheet_id:
        print("FATAL: env var 'SPREADSHEET_ID' is missing/empty.")
        sys.exit(1)

    service_account_info = json.loads(raw_service_account)
    creds_path = "credentials.json"
    
    with open(creds_path, "w") as f:
        json.dump(service_account_info, f)
    
    scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
    creds = ServiceAccountCredentials.from_json_keyfile_name(creds_path, scope)
    gc = gspread.authorize(creds)
    os.remove(creds_path)

    return gc.open_by_key(spreadsheet_id)


def _safe_col(row, idx, col_name, default=""):
    """Defensive column access: returns `default` if the column doesn't
    exist in this sheet or the row is short, instead of raising - so an
    optional field being absent never crashes the whole order load."""
    i = idx.get(col_name)
    if i is None or i >= len(row):
        return default
    return row[i]


def load_orders(spreadsheet):
    sheet = spreadsheet.worksheet(SOURCE_SHEET)
    values = sheet.get_all_values()
    if len(values) < 2: return []

    header, *data_rows = values
    idx = {name: i for i, name in enumerate(header)}
    orders = []

    for row in data_rows:
        try:
            lat = float(row[idx["Latitude Penerima"]])
            lng = float(row[idx["Longitude Penerima"]])
            box = max(0.0, float(row[idx["Total Box"]] or 0))
            if lat == 0 or lng == 0: continue

            orders.append({
                "kode_fulfillment": row[idx["Kode Fulfillment"]],
                "nama_penerima": row[idx["Nama Penerima"]],
                "alamat_penerima": row[idx["Alamat Penerima"]],
                "lat": lat, "lng": lng, "box": box,
                "tanggal_masuk": row[idx["Tanggal Fulfillment Masuk FF"]],
                # Added for the ops-team XLSX export template - each
                # defaults to "" if the column isn't present, rather than
                # crashing order load over an optional field.
                "nomor_awb": _safe_col(row, idx, "Nomor AWB"),
                "kabupaten_kota": _safe_col(row, idx, "Kabupaten/Kota Penerima"),
                "jam_masuk": _safe_col(row, idx, "Jam Fulfillment Masuk FF"),
                "jumlah_bku": _safe_col(row, idx, "Jumlah BKU", box),
            })
        except (ValueError, IndexError):
            continue
    return orders


def parse_date_flexible(value):
    if not value: return None
    for fmt in ("%m/%d/%Y", "%d/%m/%Y", "%Y-%m-%d"):
        try:
            return datetime.datetime.strptime(value.strip(), fmt).date()
        except ValueError: continue
    return None


def write_routing_output(spreadsheet, route_date, scenarios):
    """
    Writes one full plan per scenario, PRESERVING history across dates:
    re-running for the same route_date replaces just that date's rows;
    running for a different date adds alongside existing history rather
    than overwriting it. This is what lets the dashboard correctly show
    "no plan yet" for a date that's never been run, instead of always
    showing whatever the last run happened to produce.
    """
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
        ], [
            [route_date, r["vehicle"], r["driver"], r["nopol"], r["seq"], r["kode_fulfillment"],
             r["nama_penerima"], r["alamat_penerima"], r["kabupaten_kota"], r["lat"], r["lng"],
             r["box"], r["cumulative_box"], r["eta"], r["etd"],
             round(r["distance_from_prev_km"], 2), r["tanggal_masuk"],
             r["nomor_awb"], r["jam_masuk"], r["jumlah_bku"]]
            for r in result["routes"]
        ], route_date)

        _upsert_sheet_by_date(spreadsheet, backlog_sheet, [
            "Route Date", "Kode Fulfillment", "Nama Penerima", "Alamat Penerima",
            "Latitude", "Longitude", "Box", "Tanggal Fulfillment Masuk FF", "Reason",
        ], [
            [route_date, b["kode_fulfillment"], b["nama_penerima"], b["alamat_penerima"],
             b["lat"], b["lng"], b["box"], b["tanggal_masuk"], b.get("backlog_reason", "Backlogged")]
            for b in result["backlog"]
        ], route_date)

        # Real road-following geometry, fetched ONCE here while the local
        # OSRM server (started earlier in the GitHub Actions job) is still
        # up - the dashboard reads this back later instead of needing any
        # live map API call of its own, which is what made the browser-side
        # map draw straight lines instead of real roads before.
        geometry_rows = build_geometry_rows(route_date, result["routes"])
        _upsert_sheet_by_date(spreadsheet, geometry_sheet, [
            "Route Date", "Vehicle", "Geometry (JSON [[lat,lng],...])",
        ], geometry_rows, route_date)

        # Per-vehicle summary (stops, box, distance, time, utilization) was
        # already being computed for the scenario comparison totals below,
        # but never persisted anywhere the dashboard could read it directly
        # - this is what backs the per-vehicle stat cards.
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
        ])

    _upsert_sheet_by_date(spreadsheet, SCENARIOS_SHEET, [
        "Route Date", "Vehicles Available", "Vehicles Actually Used", "Stops Served",
        "Stops Backlogged", "Total Box", "Total Distance (km)",
        "Distance per Stop (km)", "See Plan Tab",
    ], comparison_rows, route_date)


def _read_sheet_rows(spreadsheet, sheet_name):
    """Returns (header, rows) as plain lists, or (None, []) if the sheet
    doesn't exist yet or has no data."""
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
    except gspread.exceptions.WorksheetNotFound:
        return None, []
    values = worksheet.get_all_values()
    if not values:
        return None, []
    return values[0], values[1:]


def _upsert_sheet_by_date(spreadsheet, sheet_name, header, new_rows, route_date, max_retries=4):
    """
    Like _write_sheet, but preserves history across dates instead of blindly
    overwriting: existing rows whose "Route Date" (first column) does NOT
    match route_date are kept as-is; rows matching route_date are replaced
    with new_rows. Re-running for the same date is idempotent (no
    duplicates); running for a new date adds alongside prior history.
    """
    existing_header, existing_rows = _read_sheet_rows(spreadsheet, sheet_name)

    if existing_header and existing_header != header:
        print(f"[{sheet_name}] WARNING: existing header doesn't match the current schema - "
              f"rebuilding this sheet from scratch (its prior history will be lost this one time).")
        existing_rows = []

    kept_rows = [row for row in existing_rows if len(row) == 0 or row[0] != route_date]
    combined_rows = kept_rows + new_rows

    return _write_sheet(spreadsheet, sheet_name, header, combined_rows, max_retries=max_retries)


def fetch_route_geometry(depot_lat, depot_lng, stops):
    """
    stops: solver-ordered list of stop dicts with 'lat'/'lng'. Returns a
    list of [lat, lng] pairs tracing the real road route (depot -> stops ->
    depot) via the local OSRM server, or None if that call fails for any
    reason - callers should fall back to straight lines for that one
    vehicle rather than fail the whole run over a single geometry lookup.
    """
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
        # GeoJSON coordinates are [lng, lat] - flip to [lat, lng] to match
        # how coordinates are handled everywhere else in this system.
        coords_geojson = data["routes"][0]["geometry"]["coordinates"]
        return [[c[1], c[0]] for c in coords_geojson]
    except Exception as e:
        print(f"  [geometry] OSRM route call failed: {e}")
        return None


def build_geometry_rows(route_date, routes):
    """routes: flat list of stop dicts (solve_routing's return), grouped
    here by vehicle. One row per vehicle that has stops AND whose OSRM
    route call succeeded; vehicles with no stops or a failed geometry
    lookup are simply omitted - the dashboard falls back to a straight
    line for those specific vehicles."""
    by_vehicle = {}
    for r in routes:
        by_vehicle.setdefault(r["vehicle"], []).append(r)

    geometry_rows = []
    for vehicle, stops in by_vehicle.items():
        stops_sorted = sorted(stops, key=lambda r: r["seq"])
        geometry = fetch_route_geometry(DEPOT_LAT, DEPOT_LNG, stops_sorted)
        if geometry:
            geometry_rows.append([route_date, vehicle, json.dumps(geometry)])
        else:
            print(f"  [geometry] no geometry captured for {vehicle} - dashboard will use a straight line for it")
    return geometry_rows


def _write_sheet(spreadsheet, sheet_name, header, rows, max_retries=4):
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
    except gspread.exceptions.WorksheetNotFound:
        worksheet = spreadsheet.add_worksheet(title=sheet_name, rows="200", cols=str(max(len(header), 1)))

    for attempt in range(1, max_retries + 1):
        try:
            worksheet.clear()
            if rows or header: worksheet.update(range_name="A1", values=[header] + rows)
            print(f"[{sheet_name}] wrote {len(rows)} row(s).")
            return True
        except gspread.exceptions.APIError as e:
            time.sleep(3 * (2 ** (attempt - 1)))
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
    
    print(f"[OSRM] Requesting {len(locations)}x{len(locations)} matrix from localhost...")
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


# Public OSRM demo server, called directly over the internet - independent
# of your self-hosted Docker/OSRM setup entirely, since it's a completely
# different service with a completely different failure mode. GitHub
# Actions runners have unrestricted internet access, so this should reach
# it even on a run where the local Docker container has an issue. It's a
# "best effort, no uptime guarantee" public service (rate-limited, and
# capped around 100 points per table query) - not something to rely on as
# the ONLY path, but genuine redundancy before falling all the way back to
# straight-line distance, which is what was producing routes that looked
# efficient "as the crow flies" but required absurd real-road detours.
OSRM_DEMO_URL = "https://router.project-osrm.org/table/v1/driving/"
OSRM_DEMO_MAX_POINTS = 100


def build_osrm_demo_matrices(locations):
    if len(locations) > OSRM_DEMO_MAX_POINTS:
        raise RuntimeError(f"{len(locations)} points exceeds the demo server's practical "
                            f"~{OSRM_DEMO_MAX_POINTS}-point limit for one table query.")

    coords = ";".join(f"{loc['lng']},{loc['lat']}" for loc in locations)
    url = OSRM_DEMO_URL + coords

    print(f"[OSRM-demo] Requesting {len(locations)}x{len(locations)} matrix from the public demo server...")
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

# Neither OSRM's default profile nor the haversine fallback accounts for
# actual traffic/congestion - both assume free-flow conditions (OSRM at
# least varies speed by road type; the haversine fallback uses one flat
# speed for everything). Rather than trying to model real traffic (which
# needs a genuinely different, paid data source - see the tradeoffs
# discussed with the dispatcher), this pads every travel-time estimate by
# a conservative multiplier, applied AFTER the real distance/time lookup so
# it's uniform regardless of source. Distance is untouched (it's a physical
# fact); only the TIME estimate - and therefore the shift-length budget and
# displayed ETAs - gets more conservative. 1.3 = assume routes take 30%
# longer than free-flow, a reasonable buffer for typical urban/suburban
# congestion without any real traffic data to calibrate against.
TRAFFIC_TIME_MULTIPLIER = _env_num("TRAFFIC_TIME_MULTIPLIER", 1.3)


def _apply_traffic_buffer(time_min):
    n = len(time_min)
    return [[time_min[i][j] * TRAFFIC_TIME_MULTIPLIER for j in range(n)] for i in range(n)]


def build_distance_time_matrices(locations):
    if os.environ.get("SKIP_OSM"):
        dist_km, time_min = build_haversine_matrices(locations)
        return dist_km, _apply_traffic_buffer(time_min)

    try:
        dist_km, time_min = build_osrm_local_matrices(locations)
        return dist_km, _apply_traffic_buffer(time_min)
    except Exception as e:
        print(f"[OSRM] Local routing server failed ({e}); trying the public OSRM demo server next.")

    try:
        dist_km, time_min = build_osrm_demo_matrices(locations)
        return dist_km, _apply_traffic_buffer(time_min)
    except Exception as e:
        print(f"[OSRM-demo] Also failed ({e}); falling back to straight-line distance.")

    dist_km, time_min = build_haversine_matrices(locations)
    return dist_km, _apply_traffic_buffer(time_min)

def solve_routing(orders, route_date, num_vehicles, dist_km, time_min):
    locations = [{"lat": DEPOT_LAT, "lng": DEPOT_LNG}] + orders
    manager = pywrapcp.RoutingIndexManager(len(locations), num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_cb(f, t): return int(round(dist_km[manager.IndexToNode(f)][manager.IndexToNode(t)] * 1000))
    def time_cb(f, t):
        i, j = manager.IndexToNode(f), manager.IndexToNode(t)
        service = service_time_minutes(locations[i].get("box", 0)) if i != 0 else 0
        return int(round(time_min[i][j] + service))
    def load_cb(f): return int(round(locations[manager.IndexToNode(f)].get("box", 0)))
    def stop_cb(f): return 0 if manager.IndexToNode(f) == 0 else 1

    dist_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)

    routing.AddDimension(routing.RegisterTransitCallback(time_cb), 0, SHIFT_LENGTH_MINUTES, True, "Time")
    routing.AddDimensionWithVehicleCapacity(routing.RegisterUnaryTransitCallback(load_cb), 0, [VEHICLE_CAPACITY_BOX] * num_vehicles, True, "Capacity")
    routing.AddDimensionWithVehicleCapacity(
        routing.RegisterUnaryTransitCallback(stop_cb), 0,
        [len(orders)] * num_vehicles, True, "StopCount")

    time_dim = routing.GetDimensionOrDie("Time")

    for idx in range(1, len(locations)):
        order_date = parse_date_flexible(locations[idx].get("tanggal_masuk"))
        age = max(0, (route_date - order_date).days) if order_date else 0
        
        # Cap age at 4 to prevent 64-bit integer overflow in OR-Tools (max ~9.2e18).
        # An H+3 order has a penalty of 10,000,000 * 100^3 = 10,000,000,000,000.
        # This mathematically guarantees the solver will drop any number of newer orders 
        # before it drops a single H+3 order.
        safe_age = min(age, 4)
        penalty = int(PRIORITY_BASE_PENALTY * (PRIORITY_EXP_MULTIPLIER ** safe_age))
        
        routing.AddDisjunction([manager.NodeToIndex(idx)], penalty)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(SOLVE_TIME_LIMIT_SECONDS)

    solution = routing.SolveWithParameters(params)
    if not solution: raise RuntimeError(f"No solution found for {num_vehicles} vehicles.")

    routes, visited, summary = [], set(), []
    for v_id in range(num_vehicles):
        idx = routing.Start(v_id)
        seq = box = dist = 0
        prev = 0
        v_route = []

        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                seq += 1; visited.add(node)
                box += locations[node].get("box", 0)
                dist += dist_km[prev][node]
                arr_min = solution.Value(time_dim.CumulVar(idx))
                stop_service_min = service_time_minutes(locations[node].get("box", 0))
                v_route.append({
                    "vehicle": VEHICLES[v_id]["name"], "driver": VEHICLES[v_id]["driver"],
                    "nopol": VEHICLES[v_id].get("nopol", ""), "seq": seq,
                    "kode_fulfillment": locations[node]["kode_fulfillment"],
                    "nama_penerima": locations[node]["nama_penerima"],
                    "alamat_penerima": locations[node]["alamat_penerima"],
                    "lat": locations[node]["lat"], "lng": locations[node]["lng"],
                    "box": locations[node].get("box", 0), "cumulative_box": box,
                    "eta": f"{(SHIFT_START_HOUR * 60 + arr_min) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + arr_min) % 60:02.0f}",
                    "etd": f"{(SHIFT_START_HOUR * 60 + arr_min + stop_service_min) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + arr_min + stop_service_min) % 60:02.0f}",
                    "distance_from_prev_km": dist_km[prev][node], "tanggal_masuk": locations[node]["tanggal_masuk"],
                    "nomor_awb": locations[node].get("nomor_awb", ""),
                    "kabupaten_kota": locations[node].get("kabupaten_kota", ""),
                    "jam_masuk": locations[node].get("jam_masuk", ""),
                    "jumlah_bku": locations[node].get("jumlah_bku", locations[node].get("box", 0)),
                })
                prev = node
            idx = solution.Value(routing.NextVar(idx))

        dist += dist_km[prev][manager.IndexToNode(idx)]
        routes.extend(v_route)
        summary.append([
            VEHICLES[v_id]["name"], VEHICLES[v_id]["driver"], seq, box, round(dist, 2),
            solution.Value(time_dim.CumulVar(idx)),
            round(100.0 * box / VEHICLE_CAPACITY_BOX, 1) if VEHICLE_CAPACITY_BOX else 0,
            f"Consider backlogging - only {seq} stops" if 0 < seq < UTILIZATION_FLAG_THRESHOLD else ""
        ])

    backlog = [{**locations[i], "backlog_reason": "Capacity/time limit"} for i in range(1, len(locations)) if i not in visited]
    return routes, backlog, summary

def _farthest_point_seeds(points, n_seeds, depot_lat, depot_lng):
    """Picks n_seeds well-spread coordinates via farthest-point sampling -
    each new seed is the point maximizing its minimum distance to all
    seeds already chosen, so seeds start out geographically spread apart
    rather than bunched together."""
    if not points:
        return []
    seeds = [max(points, key=lambda p: haversine_km(p[0], p[1], depot_lat, depot_lng))]
    while len(seeds) < n_seeds and len(seeds) < len(points):
        seeds.append(max(points, key=lambda p: min(haversine_km(p[0], p[1], s[0], s[1]) for s in seeds)))
    return seeds


def cluster_orders_by_capacity(orders, n_clusters, capacity, depot_lat, depot_lng):
    """Greedily partitions orders into n_clusters geographically-compact
    groups, each respecting `capacity`. ALWAYS returns exactly n_clusters
    lists (padded with empty lists if there aren't enough distinct order
    locations to seed that many clusters) - callers rely on one list per
    vehicle, and a vehicle with nothing assigned should show up as an
    empty/idle route, not vanish from the output entirely.

    Pure density: each order goes to its nearest cluster with capacity
    room, full stop - no stop-count balancing, no rebalancing pass. This
    maximizes geographic compactness by construction (always picks the
    closest available option), at the cost of potentially uneven stop
    counts across vehicles when order density varies by area - that
    trade-off is intentional, chosen over the balanced-but-less-compact
    version this used to be.
    """
    if n_clusters <= 1 or not orders:
        return [orders] + [[] for _ in range(max(0, n_clusters - 1))]

    points = [(o["lat"], o["lng"]) for o in orders]
    seeds = _farthest_point_seeds(points, n_clusters, depot_lat, depot_lng)
    clusters = [[] for _ in range(n_clusters)]  # always n_clusters slots, even if fewer seeds
    cluster_loads = [0.0] * len(seeds)

    for order in orders:
        candidates = sorted(
            range(len(seeds)),
            key=lambda i: haversine_km(order["lat"], order["lng"], seeds[i][0], seeds[i][1])
        )
        order_box = order.get("box", 0)
        placed = False
        for i in candidates:
            if cluster_loads[i] + order_box <= capacity:
                clusters[i].append(order)
                cluster_loads[i] += order_box
                placed = True
                break
        if not placed:
            # Every cluster is over capacity - place in the nearest one
            # regardless; the solver's own capacity dimension will
            # backlog it via the normal priority mechanism.
            best_i = candidates[0]
            clusters[best_i].append(order)
            cluster_loads[best_i] += order_box

    return clusters


def solve_clustered_scenario(orders, route_date, n, depot_lat, depot_lng):
    """Multi-vehicle scenarios: cluster orders into n compact geographic
    zones FIRST, then solve each zone as its own independent single-vehicle
    route, instead of one joint solver freely assigning any order to any
    vehicle. Reuses the exact same solve_routing() machinery per cluster -
    including its priority-based backlog mechanism - just scoped to one
    vehicle and one zone's orders at a time.

    Each cluster builds its OWN distance matrix (rather than slicing one
    matrix pre-built for the whole order set). This matters: the public
    OSRM demo server - the fallback tier used when the self-hosted OSRM
    container isn't reachable - has a roughly 100-point limit per query.
    A full day's orders can easily exceed that, but one cluster (~20-30
    orders) usually doesn't, so building matrices per-cluster is what
    actually lets that fallback tier help in practice instead of being
    skipped for real order volumes."""
    clusters = cluster_orders_by_capacity(orders, n, VEHICLE_CAPACITY_BOX, depot_lat, depot_lng)

    all_routes, all_backlog, all_summary = [], [], []
    for cluster_idx, cluster_orders in enumerate(clusters):
        if cluster_idx >= NUM_VEHICLES:
            all_backlog.extend([{**o, "backlog_reason": "No vehicle available for this cluster"} for o in cluster_orders])
            continue

        vehicle = VEHICLES[cluster_idx]
        if not cluster_orders:
            all_summary.append([vehicle["name"], vehicle["driver"], 0, 0, 0.0, 0, 0, ""])
            continue

        cluster_locations = [{"lat": depot_lat, "lng": depot_lng}] + cluster_orders
        sub_dist, sub_time = build_distance_time_matrices(cluster_locations)

        routes, backlog, summary = solve_routing(cluster_orders, route_date, 1, sub_dist, sub_time)

        # solve_routing always labels a lone vehicle as VEHICLES[0]
        # internally - relabel to this cluster's actual assigned vehicle.
        for r in routes:
            r["vehicle"] = vehicle["name"]
            r["driver"] = vehicle["driver"]
            r["nopol"] = vehicle.get("nopol", "")
        for s in summary:
            s[0] = vehicle["name"]
            s[1] = vehicle["driver"]

        all_routes.extend(routes)
        all_backlog.extend(backlog)
        all_summary.extend(summary)

    return all_routes, all_backlog, all_summary


def solve_all_scenarios(orders, route_date):
    scenarios = {}
    for n in range(1, NUM_VEHICLES + 1):
        print(f"\n--- Scenario: {n} vehicle(s) ---")
        try:
            if n == 1:
                locations = [{"lat": DEPOT_LAT, "lng": DEPOT_LNG}] + orders
                dist_km, time_min = build_distance_time_matrices(locations)
                r, b, s = solve_routing(orders, route_date, 1, dist_km, time_min)
            else:
                r, b, s = solve_clustered_scenario(orders, route_date, n, DEPOT_LAT, DEPOT_LNG)
            print(f"  Served {sum(row[2] for row in s)}/{len(orders)}, backlog {len(b)}, dist {sum(row[4] for row in s):.1f}km")
            scenarios[n] = {"routes": r, "backlog": b, "summary": s}
        except Exception as e:
            scenarios[n] = {"error": str(e)}
    return scenarios

def main():
    route_date_str = os.environ.get("ROUTE_DATE", "").strip()
    route_date = datetime.datetime.strptime(route_date_str, "%Y-%m-%d").date() if route_date_str else datetime.date.today() + datetime.timedelta(days=1)
    
    print(f"Routing for date: {route_date.isoformat()}")
    spreadsheet = get_spreadsheet()
    orders = load_orders(spreadsheet)
    if not orders: return print("No orders to route.")

    scenarios = solve_all_scenarios(orders, route_date)
    write_routing_output(spreadsheet, route_date.strftime("%d/%m/%Y"), scenarios)

if __name__ == "__main__":
    main()
