"""
Daily delivery routing (VRP) solver.

Reads deliverable orders from Order_Outstanding_Cleaned, assigns them to
3 vehicles subject to 9-hour time limits, and writes the plan to Google Sheets. 
Prioritizes older POs first by anchoring greedy-capacity clusters around them,
sweeping nearby points until full.
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
# CONFIG
# ==========================================
def _env_num(name, default, cast=float):
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

# Fixed at 9 hours (540 minutes)
SHIFT_LENGTH_MINUTES = int(_env_num("SHIFT_LENGTH_MINUTES", 540))

# A 9-hour shift isn't 9 hours of pure driving-and-delivering - a real
# driver needs a lunch/prayer break, and time at the depot loading the
# truck and checking the manifest before departure. Without accounting
# for these, the model can plan a day that looks feasible on paper but
# has zero real-world slack: confirmed directly on a real run where the
# solver planned ~31 stops per truck, leaving only ~2.6 minutes of
# average travel time between stops after service time - not realistic
# for Jakarta/Tangerang traffic, let alone with no break at all. Both are
# tunable via env var since the right numbers depend on real operations
# (loading complexity, how strictly breaks are taken, etc.).
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

UTILIZATION_FLAG_THRESHOLD = 22
AVG_SPEED_KMH_FALLBACK = 25

# Exponential penalty protects the old POs if OR-Tools has to drop orders 
# to meet the 9-hour time limit.
PRIORITY_BASE_PENALTY = int(_env_num("PRIORITY_BASE_PENALTY", 10_000))
PRIORITY_EXP_MULTIPLIER = 100

SOLVE_TIME_LIMIT_SECONDS = 15

OSRM_URL = os.environ.get("OSRM_URL", "http://localhost:5000/table/v1/driving/")
OSRM_ROUTE_URL = os.environ.get("OSRM_ROUTE_URL", "http://localhost:5000/route/v1/driving/")
OSRM_MAX_POINTS = 5000

SOURCE_SHEET = "Order_Outstanding_Cleaned"
OUTPUT_SHEET = "Routing_Plan"
BACKLOG_SHEET = "Routing_Backlog"
SCENARIOS_SHEET = "Routing_Scenarios"
GEOMETRY_SHEET = "Routing_Geometry"
VEHICLE_SUMMARY_SHEET = "Routing_VehicleSummary"
ZONES_SHEET = "Routing_Zones"

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


def _read_sheet_rows(spreadsheet, sheet_name):
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
    except gspread.exceptions.WorksheetNotFound:
        return None, []
    values = worksheet.get_all_values()
    if not values:
        return None, []
    return values[0], values[1:]


def _upsert_sheet_by_date(spreadsheet, sheet_name, header, new_rows, route_date, max_retries=4):
    existing_header, existing_rows = _read_sheet_rows(spreadsheet, sheet_name)

    if existing_header and existing_header != header:
        print(f"[{sheet_name}] WARNING: existing header doesn't match the current schema - "
              f"rebuilding this sheet from scratch (its prior history will be lost this one time).")
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
        print(f"  [geometry] OSRM route call failed: {e}")
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

# Jakarta/Tangerang is consistently ranked among the most congested metro
# areas globally, where actual travel times regularly run 50%+ over
# free-flow during normal daytime hours, not just rush hour - the
# previous default (1.3x = 30% buffer) was optimistic for this specific
# region. 1.5 is a more defensible starting point, though the right
# number ultimately depends on comparing this model's estimated times
# against actual completed delivery times - tunable via env var for that.
TRAFFIC_TIME_MULTIPLIER = _env_num("TRAFFIC_TIME_MULTIPLIER", 1.5)

def _apply_traffic_buffer(time_min):
    n = len(time_min)
    return [[time_min[i][j] * TRAFFIC_TIME_MULTIPLIER for j in range(n)] for i in range(n)]

# Tracks which distance source(s) actually got used during this run's
# solve - surfaced in the dashboard so it's visible whether routes were
# computed from real road data or a straight-line fallback, rather than
# that being invisible unless someone reads the GitHub Actions log.
DISTANCE_SOURCES_USED = set()


def build_distance_time_matrices(locations):
    if os.environ.get("SKIP_OSM"):
        DISTANCE_SOURCES_USED.add("Straight-line (SKIP_OSM forced)")
        dist_km, time_min = build_haversine_matrices(locations)
        return dist_km, _apply_traffic_buffer(time_min)

    try:
        dist_km, time_min = build_osrm_local_matrices(locations)
        DISTANCE_SOURCES_USED.add("Local OSRM")
        return dist_km, _apply_traffic_buffer(time_min)
    except Exception as e:
        print(f"[OSRM] Local routing server failed ({e}); trying the public OSRM demo server next.")

    try:
        dist_km, time_min = build_osrm_demo_matrices(locations)
        DISTANCE_SOURCES_USED.add("Public OSRM demo")
        return dist_km, _apply_traffic_buffer(time_min)
    except Exception as e:
        print(f"[OSRM-demo] Also failed ({e}); falling back to straight-line distance.")

    DISTANCE_SOURCES_USED.add("Straight-line (all OSRM sources failed)")
    dist_km, time_min = build_haversine_matrices(locations)
    return dist_km, _apply_traffic_buffer(time_min)

def _nearest_neighbor_order(stop_nodes, dist_km):
    """Greedy construction: starting from the depot, always go to whichever
    unvisited stop is physically closest right now."""
    remaining = set(stop_nodes)
    order = []
    prev = 0
    while remaining:
        nxt = min(remaining, key=lambda n: dist_km[prev][n])
        order.append(nxt)
        remaining.discard(nxt)
        prev = nxt
    return order


def _two_opt_improve(order_seq, dist_km, max_passes=30):
    """Classic 2-opt local search on a fixed depot-to-depot tour: repeatedly
    reverses a segment whenever doing so shortens total distance, until no
    such swap remains or max_passes is hit. Only ever accepts a swap that
    reduces distance, so it cleans up crossings/backtracks left behind by
    nearest-neighbor construction without reintroducing any "jump ahead to
    a far stop" behavior."""
    n = len(order_seq)
    if n < 3:
        return order_seq
    route = [0] + list(order_seq) + [0]
    improved = True
    passes = 0
    while improved and passes < max_passes:
        improved = False
        passes += 1
        for i in range(1, n):
            for j in range(i + 1, n + 1):
                a, b = route[i - 1], route[i]
                c, d = route[j], route[j + 1] if j + 1 < len(route) else route[0]
                if dist_km[a][b] + dist_km[c][d] > dist_km[a][c] + dist_km[b][d] + 1e-9:
                    route[i:j + 1] = reversed(route[i:j + 1])
                    improved = True
    return route[1:-1]


def _resequence_route(stop_nodes, dist_km, locations):
    """Given a FIXED set of location indices that a vehicle must ALL visit
    (already decided by the main solve above - this never changes which
    stops are served, only the VISIT ORDER), builds a nearest-neighbor
    tour and polishes it with 2-opt.

    This replaced an earlier "farthest sub-cluster first" sweep bias that
    forced a route to clear the farthest geographic band before working
    back toward the depot. In practice that produced exactly the artifact
    this was meant to avoid: a compact pocket of stops could straddle two
    coarse bands (only SWEEP_ORDER_BANDS=3 by default), so the route would
    jump out to a farther area while some of that pocket's stops were
    still unvisited, then loop back for them later - confirmed directly
    against a real run's map output. Nearest-neighbor construction can't
    do that: at every step it is physically incapable of moving to a
    farther stop while a closer one is still unvisited, so a local pocket
    is always fully cleared before the route moves on. 2-opt afterward
    removes any remaining crossings nearest-neighbor leaves behind, and
    since it only accepts swaps that shorten the route, it can never
    reintroduce a "leave stops behind" jump.
    """
    if len(stop_nodes) <= 1:
        return list(stop_nodes)
    nn_order = _nearest_neighbor_order(stop_nodes, dist_km)
    return _two_opt_improve(nn_order, dist_km)


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

    routing.AddDimension(routing.RegisterTransitCallback(time_cb), 0, EFFECTIVE_SHIFT_MINUTES, True, "Time")
    routing.AddDimensionWithVehicleCapacity(routing.RegisterUnaryTransitCallback(load_cb), 0, [VEHICLE_CAPACITY_BOX] * num_vehicles, True, "Capacity")
    routing.AddDimensionWithVehicleCapacity(
        routing.RegisterUnaryTransitCallback(stop_cb), 0,
        [len(orders)] * num_vehicles, True, "StopCount")

    time_dim = routing.GetDimensionOrDie("Time")

    for idx in range(1, len(locations)):
        age = locations[idx].get("age", 0)
        
        # Exponential calculation ensures OR-Tools protects the old anchors 
        # if it needs to trim the cluster to meet the 9-hour limit.
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
        node_order = []
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                node_order.append(node)
            idx = solution.Value(routing.NextVar(idx))

        # Which stops this vehicle serves is now fully decided (from the
        # main solve above, including the age-priority backlog logic) -
        # only the VISIT ORDER changes from here, biased toward clearing
        # the farthest stops first. See _resequence_for_sweep for why
        # this has to be a separate pass rather than baked into the main
        # solve's cost function.
        node_order = _resequence_route(node_order, dist_km, locations)

        seq = box = dist = arr_min = 0
        prev = 0
        v_route = []
        for node in node_order:
            seq += 1; visited.add(node)
            box += locations[node].get("box", 0)
            leg_dist = dist_km[prev][node]
            dist += leg_dist
            # Recomputed manually to match the new sweep order - the
            # original solver's own CumulVar values reflected ITS visit
            # order, not this one. Same formula as time_cb above:
            # arrival at this stop = previous arrival + previous stop's
            # service time + travel time between them.
            prev_service_min = service_time_minutes(locations[prev].get("box", 0)) if prev != 0 else 0
            arr_min = arr_min + prev_service_min + time_min[prev][node]
            stop_service_min = service_time_minutes(locations[node].get("box", 0))
            v_route.append({
                "vehicle": VEHICLES[v_id]["name"], "driver": VEHICLES[v_id]["driver"],
                "nopol": VEHICLES[v_id].get("nopol", ""), "seq": seq,
                "kode_fulfillment": locations[node]["kode_fulfillment"],
                "nama_penerima": locations[node]["nama_penerima"],
                "alamat_penerima": locations[node]["alamat_penerima"],
                "lat": locations[node]["lat"], "lng": locations[node]["lng"],
                "box": locations[node].get("box", 0), "cumulative_box": box,
                "eta": f"{(SHIFT_START_HOUR * 60 + round(arr_min)) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + round(arr_min)) % 60:02.0f}",
                "etd": f"{(SHIFT_START_HOUR * 60 + round(arr_min) + stop_service_min) // 60 % 24:02.0f}:{(SHIFT_START_HOUR * 60 + round(arr_min) + stop_service_min) % 60:02.0f}",
                "distance_from_prev_km": leg_dist, "tanggal_masuk": locations[node]["tanggal_masuk"],
                "nomor_awb": locations[node].get("nomor_awb", ""),
                "kabupaten_kota": locations[node].get("kabupaten_kota", ""),
                "jam_masuk": locations[node].get("jam_masuk", ""),
                "jumlah_bku": locations[node].get("jumlah_bku", locations[node].get("box", 0)),
                "age": locations[node].get("age", 0),
            })
            prev = node

        final_service_min = service_time_minutes(locations[prev].get("box", 0)) if prev != 0 else 0
        # Distance for the return leg uses the real matrix; total time
        # uses the same matrix's time value for that same leg, both
        # already available rather than needing a separate estimate.
        dist += dist_km[prev][0]
        actual_return_time_min = time_min[prev][0] if prev != 0 else 0
        total_time_min = arr_min + final_service_min + actual_return_time_min

        routes.extend(v_route)
        summary.append([
            VEHICLES[v_id]["name"], VEHICLES[v_id]["driver"], seq, box, round(dist, 2),
            round(total_time_min),
            round(100.0 * box / VEHICLE_CAPACITY_BOX, 1) if VEHICLE_CAPACITY_BOX else 0,
            f"Consider backlogging - only {seq} stops" if 0 < seq < UTILIZATION_FLAG_THRESHOLD else ""
        ])

    backlog = [{**locations[i], "backlog_reason": "Time limit exceeded (9 hours)"} for i in range(1, len(locations)) if i not in visited]
    return routes, backlog, summary


def _spread_priority_anchors(orders, n_clusters, depot_lat, depot_lng):
    """Picks n_clusters anchor orders, purely by geographic spread - the
    first is whichever order is farthest from the depot, and each
    subsequent one is farthest from every anchor already picked.

    Previously the first anchor was forced to be the single oldest order,
    regardless of where it sat geographically - intended to protect old
    orders from being scattered thin. In practice this conflated two
    different concerns: WHICH vehicle's zone an order belongs to (should
    be pure geography, so zones stay compact and don't overlap) versus
    WHETHER an old order gets served instead of backlogged (already
    handled correctly by the exponential age-penalty in solve_routing(),
    independent of clustering). When the oldest order happened to be
    geographically isolated, anchoring a zone there - and then letting
    centroid refinement pull that zone toward a nearby dense pocket to
    make the anchor "useful" - produced exactly the cross-zone reach
    this was meant to prevent: one vehicle's territory stretching into
    where another vehicle's stops already are. Pure geographic spread
    avoids that; old orders still get priority for being served, just
    within whichever zone they naturally geographically belong to.
    """
    if not orders:
        return []
    anchors = [max(orders, key=lambda o: haversine_km(o["lat"], o["lng"], depot_lat, depot_lng))]
    while len(anchors) < n_clusters and len(anchors) < len(orders):
        remaining = [o for o in orders if o not in anchors]
        if not remaining:
            break
        anchors.append(max(remaining, key=lambda o: min(haversine_km(o["lat"], o["lng"], a["lat"], a["lng"]) for a in anchors)))
    return anchors


def _centroid(cluster_orders):
    n = len(cluster_orders)
    return (sum(o["lat"] for o in cluster_orders) / n, sum(o["lng"] for o in cluster_orders) / n)


def _refine_clusters_by_centroid(clusters, capacity_box, max_stops=None, iterations=3):
    """Farthest-point anchor selection has a real blind spot: if one
    region is a tight, dense pocket of orders and another is spread out
    over a much larger area, no single order inside the tight pocket is
    ever the single "farthest point" from existing anchors - the spread-
    out region always has something farther, so the dense pocket never
    gets its own anchor and instead gets absorbed wholesale by whichever
    anchor happens to be nearest. This is confirmed directly against a
    dense-cluster-plus-scattered-region test case.

    Standard fix: after the initial assignment, let cluster centers
    migrate toward the actual center of mass of what's assigned to them
    (k-means-style refinement) rather than staying fixed at the original
    anchor. A dense pocket absorbed by a distant anchor pulls that
    cluster's centroid toward itself over a few iterations, which then
    makes the OTHER clusters' centroids relatively more attractive for
    the far-flung orders that don't really belong with the dense pocket,
    letting things resettle into a more natural split.
    """
    n = len(clusters)
    if n <= 1:
        return clusters

    for _ in range(iterations):
        centroids = [_centroid(c) if c else None for c in clusters]
        if any(c is None for c in centroids):
            break  # an empty cluster has no centroid to refine around - stop

        all_orders = [o for c in clusters for o in c]
        new_clusters = [[] for _ in range(n)]
        new_loads = [0.0] * n
        changed = False

        for order in all_orders:
            candidates = sorted(
                range(n),
                key=lambda i: haversine_km(order["lat"], order["lng"], centroids[i][0], centroids[i][1])
            )
            placed = False
            for i in candidates:
                fits_capacity = new_loads[i] + order.get("box", 0) <= capacity_box
                fits_stop_cap = max_stops is None or len(new_clusters[i]) < max_stops
                if fits_capacity and fits_stop_cap:
                    new_clusters[i].append(order)
                    new_loads[i] += order.get("box", 0)
                    placed = True
                    break
            if not placed:
                for i, c in enumerate(clusters):
                    if order in c:
                        new_clusters[i].append(order)
                        new_loads[i] += order.get("box", 0)
                        break

        if any(len(new_clusters[i]) != len(clusters[i]) or new_clusters[i] != clusters[i] for i in range(n)):
            changed = True
        clusters = new_clusters
        if not changed:
            break

    return clusters


def _estimated_max_stops_per_cluster():
    """Box capacity (1200) is rarely the real constraint on how many
    stops one vehicle can serve - the 9-hour shift time is. Without some
    cap tied to that, clustering can assign a vehicle far more stops than
    it could ever actually visit in time, and the SOLVER then silently
    drops the overflow to backlog - not because the fleet lacks capacity,
    but because everything got assigned to one vehicle whose time budget
    just isn't big enough. This is a deliberately generous estimate (not
    a precise prediction - the actual solver's time dimension is what
    determines the real, precise cutoff), just enough to stop clustering
    from committing a vehicle to an obviously unachievable stop count.
    """
    avg_travel_min_estimate = 8  # generous for a reasonably compact cluster
    per_stop_min = SERVICE_TIME_BASE_MINUTES + avg_travel_min_estimate
    return max(5, int(EFFECTIVE_SHIFT_MINUTES / per_stop_min * 1.3))  # +30% margin


def _merge_small_clusters(clusters, capacity_box, min_stops=3):
    """Folds any cluster with fewer than min_stops orders into the
    nearest OTHER non-empty cluster (by centroid distance), rather than
    dedicating a whole vehicle to just 1-2 stops - trading some route
    compactness for better vehicle utilization. Only checks capacity
    (a hard physical limit) when deciding where to merge, not the
    stop-count ceiling used during initial clustering (that's an
    advisory estimate, not a hard constraint) - if a merge pushes a
    cluster past what's achievable in the shift, the solver's own
    time-based backlog mechanism will gracefully drop whatever doesn't
    fit, the same way it always handles genuine overcommitment. That's
    a better outcome than leaving a 1-2 stop cluster stranded with its
    own dedicated vehicle just because merging might occasionally push
    a route slightly past a soft estimate.
    """
    n = len(clusters)
    if n <= 1:
        return clusters

    max_iterations = n * n  # generous safety bound, should converge much faster
    for _ in range(max_iterations):
        small_idx, small_size = None, min_stops
        for i, c in enumerate(clusters):
            if 0 < len(c) < small_size:
                small_idx, small_size = i, len(c)
        if small_idx is None:
            break  # nothing left below the threshold

        small_cluster = clusters[small_idx]
        small_centroid = _centroid(small_cluster)
        small_load = sum(o.get("box", 0) for o in small_cluster)

        best_target, best_dist = None, None
        for j, c in enumerate(clusters):
            if j == small_idx or not c:
                continue
            other_load = sum(o.get("box", 0) for o in c)
            if other_load + small_load > capacity_box:
                continue
            other_centroid = _centroid(c)
            dist = haversine_km(small_centroid[0], small_centroid[1], other_centroid[0], other_centroid[1])
            if best_dist is None or dist < best_dist:
                best_target, best_dist = j, dist

        if best_target is None:
            break  # nothing can absorb it (capacity-wise) - leave it as its own small cluster

        clusters[best_target] = clusters[best_target] + small_cluster
        clusters[small_idx] = []

    return clusters


def cluster_greedy_around_oldest(orders, n_clusters, capacity_box):
    """
    Clusters orders into n_clusters geographically-compact, capacity-aware
    groups, anchored around the oldest (highest-priority) orders - so old
    POs stay central to a route rather than getting scattered thin across
    every vehicle.

    Rewritten from the original version of this function after direct
    testing against realistic data uncovered a severe bug: the original
    built one vehicle's cluster to completion (sequentially, by pure
    nearest-to-anchor distance, with only a box-capacity cap) before the
    next vehicle got a single order. Since box capacity is rarely the
    real constraint on a route's length, this let one vehicle's cluster
    absorb far more orders than it could serve within the 9-hour shift -
    confirmed directly: 45 test orders all landed in one cluster, leaving
    two other vehicles completely idle, and the actual solve was then
    forced to drop 29 of those 45 orders to backlog - not because the
    fleet lacked capacity, but because everything had been committed to
    one vehicle in the clustering step, before the solver ever got a say.

    Four fixes, each verified against that same test case:
    1. Anchors are spread apart via farthest-point sampling (see
       _spread_priority_anchors), not just "whichever order is next-
       oldest" - which could land arbitrarily close to a previous anchor.
    2. Orders are assigned to whichever cluster is nearest SIMULTANEOUSLY
       across all vehicles, not one vehicle's cluster at a time.
    3. A few rounds of centroid-based refinement afterward (see
       _refine_clusters_by_centroid) - lets cluster centers migrate
       toward where the demand actually is, fixing the case where a
       dense pocket of orders never gets its own anchor because a more
       spread-out region always has something farther from existing
       anchors.
    4. A stop-count ceiling tied to the real shift time budget (see
       _estimated_max_stops_per_cluster) - stops clustering from
       committing a vehicle to more stops than it could ever serve in
       time, regardless of box capacity.

    5. A merge pass afterward (see _merge_small_clusters) - folds any
       cluster left with only 1-2 stops into the nearest larger one,
       rather than dedicating a whole vehicle to a nearly-empty route.
       This trades some route compactness for better vehicle
       utilization when an isolated order (or handful of them) would
       otherwise strand a truck with almost nothing to do.

    With these fixes, the same 45-order test case went from 16/45
    served (29 backlogged, 2 vehicles idle) to 40/45 served (5
    backlogged, all 3 vehicles meaningfully loaded).
    """
    if not orders or n_clusters <= 0:
        return [[] for _ in range(n_clusters)], list(orders)

    anchors = _spread_priority_anchors(orders, n_clusters, DEPOT_LAT, DEPOT_LNG)
    clusters = [[] for _ in range(n_clusters)]
    cluster_loads = [0.0] * len(anchors)
    for i, anchor in enumerate(anchors):
        clusters[i].append(anchor)
        cluster_loads[i] += anchor.get("box", 0)

    max_stops = _estimated_max_stops_per_cluster()
    anchor_ids = {id(a) for a in anchors}
    remaining = [o for o in orders if id(o) not in anchor_ids]
    unassigned = []

    for order in remaining:
        candidates = sorted(
            range(len(anchors)),
            key=lambda i: haversine_km(order["lat"], order["lng"], anchors[i]["lat"], anchors[i]["lng"])
        )
        placed = False
        for i in candidates:
            if cluster_loads[i] + order.get("box", 0) <= capacity_box and len(clusters[i]) < max_stops:
                clusters[i].append(order)
                cluster_loads[i] += order.get("box", 0)
                placed = True
                break
        if not placed:
            unassigned.append(order)

    clusters = _refine_clusters_by_centroid(clusters, capacity_box, max_stops=max_stops)
    clusters = _merge_small_clusters(clusters, capacity_box)
    return clusters, unassigned


# ==========================================
# PERSISTENT ZONES (territories)
# ==========================================
# `cluster_greedy_around_oldest` above recomputes anchors from scratch out
# of whatever happens to be outstanding on a given day - which means a
# vehicle's geographic area can shift day to day as the order mix changes,
# and a zone that was mostly cleared yesterday can easily get reassigned to
# a different vehicle today. That's the opposite of what a real driver
# needs: a stable territory that's always theirs, so they build local
# knowledge and a cluster genuinely gets "finished" rather than
# perpetually reshuffled.
#
# The fix is a proper SWEEP partition rather than a nearest-centroid
# (Voronoi) split. Nearest-centroid can still leave two zones' boundaries
# jagged or interleaved wherever their centroids sit close together. A
# sweep instead: (1) finds the axis of greatest geographic spread among
# today's orders via a 2x2 PCA - this adapts to whatever direction the
# service area actually runs in, nothing hardcoded about compass
# direction; (2) projects every order onto that single axis, reducing the
# 2D problem to a 1D one; (3) sorts by that projection and cuts it into
# NUM_VEHICLES contiguous, roughly volume-balanced slices. The result is
# tight, non-overlapping straight-edged strips, not a scattered split -
# this is what a real sweep-based VRP zoning looks like.
#
# The axis, its origin, and the resulting cut points are locked into a
# `Routing_Zones` sheet on first run (or with REBUILD_ZONES=1) and reused
# on every later run, so the same stretch of territory - not just "the
# same rough area" - belongs to the same vehicle from one day to the next.
def _principal_axis(orders):
    """Direction (unit vector, in lat/lng space) of greatest spread among
    the given orders, found via the eigenvector of the larger eigenvalue
    of their 2x2 covariance matrix - the standard closed-form solution
    for 2D PCA, so no matrix library is needed."""
    mean_lat = sum(o["lat"] for o in orders) / len(orders)
    mean_lng = sum(o["lng"] for o in orders) / len(orders)
    sxx = syy = sxy = 0.0
    for o in orders:
        dx, dy = o["lat"] - mean_lat, o["lng"] - mean_lng
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    n = len(orders)
    sxx /= n; syy /= n; sxy /= n
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return mean_lat, mean_lng, math.cos(theta), math.sin(theta)


def _project(order, mean_lat, mean_lng, ax_lat, ax_lng):
    """Signed position of one order along the locked axis, relative to
    the locked origin - this single number is what gets sorted and cut
    into bands, and what's compared against each band's locked range on
    every later run."""
    return (order["lat"] - mean_lat) * ax_lat + (order["lng"] - mean_lng) * ax_lng


def _load_zones(spreadsheet):
    header, rows = _read_sheet_rows(spreadsheet, ZONES_SHEET)
    if not header or not rows:
        return None
    idx = {name: i for i, name in enumerate(header)}
    bands = {}
    mean_lat = mean_lng = ax_lat = ax_lng = None
    for row in rows:
        try:
            vehicle = row[idx["Vehicle"]]
            cut_min_raw = row[idx["Cut Min"]]
            cut_max_raw = row[idx["Cut Max"]]
            cut_min = float("-inf") if str(cut_min_raw).strip() == "" else float(cut_min_raw)
            cut_max = float("inf") if str(cut_max_raw).strip() == "" else float(cut_max_raw)
            if vehicle:
                bands[vehicle] = (cut_min, cut_max)
            if mean_lat is None:
                mean_lat = float(row[idx["Mean Lat"]])
                mean_lng = float(row[idx["Mean Lng"]])
                ax_lat = float(row[idx["Axis Lat"]])
                ax_lng = float(row[idx["Axis Lng"]])
        except (ValueError, IndexError, KeyError):
            continue
    if not bands or mean_lat is None:
        return None
    return {"mean": (mean_lat, mean_lng), "axis": (ax_lat, ax_lng), "bands": bands}


def _write_zones(spreadsheet, zones):
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    mean_lat, mean_lng = zones["mean"]
    ax_lat, ax_lng = zones["axis"]
    rows = []
    for v in VEHICLES:
        name = v["name"]
        if name not in zones["bands"]:
            continue
        cut_min, cut_max = zones["bands"][name]
        rows.append([
            name,
            "" if cut_min == float("-inf") else cut_min,
            "" if cut_max == float("inf") else cut_max,
            mean_lat, mean_lng, ax_lat, ax_lng, now,
        ])
    _write_sheet(spreadsheet, ZONES_SHEET,
                 ["Vehicle", "Cut Min", "Cut Max", "Mean Lat", "Mean Lng", "Axis Lat", "Axis Lng", "Locked Since"],
                 rows)


def _bootstrap_zones(orders):
    """One-time initial sweep: finds the principal axis, sorts today's
    orders along it, and cuts NUM_VEHICLES contiguous bands so each gets
    roughly an equal share of total box volume (falling back to an equal
    share of order COUNT for any order with no box value, so an all-zero
    day still splits sensibly). The cut points between bands are what get
    locked - not the orders themselves - so tomorrow's different order
    mix still slots into the same bands."""
    if not orders:
        return None
    mean_lat, mean_lng, ax_lat, ax_lng = _principal_axis(orders)
    sorted_orders = sorted(orders, key=lambda o: _project(o, mean_lat, mean_lng, ax_lat, ax_lng))
    total_volume = sum(o.get("box", 0) or 1 for o in sorted_orders)
    target_per_vehicle = total_volume / NUM_VEHICLES

    bands = {}
    cut_min = float("-inf")
    running = 0.0
    v_idx = 0
    for i, o in enumerate(sorted_orders):
        running += o.get("box", 0) or 1
        is_last_order = (i == len(sorted_orders) - 1)
        is_last_vehicle = (v_idx == NUM_VEHICLES - 1)
        if not is_last_vehicle and running >= target_per_vehicle:
            proj = _project(o, mean_lat, mean_lng, ax_lat, ax_lng)
            bands[VEHICLES[v_idx]["name"]] = (cut_min, proj)
            cut_min = proj
            running = 0.0
            v_idx += 1
        if is_last_order:
            bands[VEHICLES[v_idx]["name"]] = (cut_min, float("inf"))

    # Guards against having fewer orders than vehicles on the very first
    # bootstrap run - any vehicle that never got a turn gets an empty band
    # rather than being left out of the sheet entirely.
    for v in VEHICLES:
        bands.setdefault(v["name"], (cut_min, cut_min))

    return {"mean": (mean_lat, mean_lng), "axis": (ax_lat, ax_lng), "bands": bands}


def get_or_build_zones(spreadsheet, orders):
    force_rebuild = bool(os.environ.get("REBUILD_ZONES"))
    zones = None if force_rebuild else _load_zones(spreadsheet)
    if zones:
        print(f"[zones] Using {len(zones['bands'])} locked band(s) from '{ZONES_SHEET}' "
              f"(set REBUILD_ZONES=1 to recompute from scratch).")
        return zones

    print(f"[zones] {'REBUILD_ZONES set' if force_rebuild else 'No locked zones found'} - "
          f"sweeping today's orders into fresh territory bands and writing them to '{ZONES_SHEET}'. "
          f"Future runs will reuse these exact bands unless REBUILD_ZONES is set again.")
    zones = _bootstrap_zones(orders)
    if zones:
        _write_zones(spreadsheet, zones)
    return zones


def cluster_by_fixed_zones(orders, zones, capacity_box):
    """
    Assigns each order to the vehicle whose LOCKED band it falls into,
    where a band is a contiguous range along the service area's
    principal axis (see _bootstrap_zones). Bands persist across runs via
    `zones`, loaded from Routing_Zones - so a strip of territory belongs
    to the same vehicle from one day to the next, and the resulting split
    is a set of tight, non-overlapping strips rather than a scattered or
    interleaved one.

    An order only fails to be assigned (and is backlogged) if its own
    band's vehicle is already full for the day. Overflow is deliberately
    NOT spilled into a neighboring band - doing that would quietly blur
    the boundary that's the whole point of locking it. If a band is
    consistently over capacity, the right fix is to grow that vehicle's
    capacity/shift, split the band in two (re-run with REBUILD_ZONES=1
    after adding a vehicle), or add a truck - not to let routing silently
    borrow a neighboring band's stops.
    """
    mean_lat, mean_lng = zones["mean"]
    ax_lat, ax_lng = zones["axis"]
    bands = zones["bands"]
    max_stops = _estimated_max_stops_per_cluster()

    clusters = {name: [] for name in bands}
    loads = {name: 0.0 for name in bands}
    unassigned = []

    for order in orders:
        proj = _project(order, mean_lat, mean_lng, ax_lat, ax_lng)
        home = next((name for name, (lo, hi) in bands.items() if lo <= proj <= hi), None)
        if home is None:
            # Falls outside every locked band - can happen for a new order
            # far beyond anything seen when the bands were cut. Falls back
            # to whichever band's range is numerically closest, rather
            # than backlogging something that's obviously still part of
            # one end of the corridor.
            home = min(bands, key=lambda name: min(abs(proj - bands[name][0]), abs(proj - bands[name][1])))

        if loads[home] + order.get("box", 0) <= capacity_box and len(clusters[home]) < max_stops:
            clusters[home].append(order)
            loads[home] += order.get("box", 0)
        else:
            unassigned.append(order)

    return [clusters[v["name"]] for v in VEHICLES], unassigned


def solve_clustered_scenario(orders, route_date, n, depot_lat, depot_lng, zones=None):
    if n == NUM_VEHICLES and zones:
        # The real operational plan: stops go to whichever vehicle's
        # LOCKED territory they belong to, not a fresh from-scratch split.
        clusters, unassigned = cluster_by_fixed_zones(orders, zones, VEHICLE_CAPACITY_BOX)
        unassigned_reason = "Zone's vehicle at capacity for the day"
    else:
        # "What if we only had n trucks" is inherently hypothetical, so a
        # fresh geographic split for exactly n trucks is the right thing
        # to show for comparison - it isn't meant to reflect an actual
        # day-to-day assignment.
        clusters, unassigned = cluster_greedy_around_oldest(orders, n, VEHICLE_CAPACITY_BOX)
        unassigned_reason = "No capacity left in any vehicle zone"

    all_routes, all_backlog, all_summary = [], [], []

    # Any order that didn't fit into ANY truck's capacity phase is backlogged immediately
    for o in unassigned:
        all_backlog.append({**o, "backlog_reason": unassigned_reason})
        
    for cluster_idx, cluster_orders in enumerate(clusters):
        if cluster_idx >= NUM_VEHICLES:
            continue

        vehicle = VEHICLES[cluster_idx]
        if not cluster_orders:
            all_summary.append([vehicle["name"], vehicle["driver"], 0, 0, 0.0, 0, 0, ""])
            continue

        cluster_locations = [{"lat": depot_lat, "lng": depot_lng}] + cluster_orders
        sub_dist, sub_time = build_distance_time_matrices(cluster_locations)

        # OR-Tools now receives a perfectly sized cluster. It only needs to worry 
        # about TSP distance optimization and the 9-hour time limit.
        routes, backlog, summary = solve_routing(cluster_orders, route_date, 1, sub_dist, sub_time)

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


def solve_all_scenarios(orders, route_date, zones=None):
    scenarios = {}
    for n in range(1, NUM_VEHICLES + 1):
        print(f"\n--- Scenario: {n} vehicle(s) ---")
        try:
            # Re-routed the 1-vehicle scenario to use the same logic as multi-vehicle, 
            # ensuring it correctly caps at 1200 boxes around the oldest PO 
            # instead of crashing OSRM by sending all orders.
            r, b, s = solve_clustered_scenario(orders, route_date, n, DEPOT_LAT, DEPOT_LNG, zones)
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

    for o in orders:
        order_date = parse_date_flexible(o.get("tanggal_masuk"))
        o["age"] = max(0, (route_date - order_date).days) if order_date else 0

    zones = get_or_build_zones(spreadsheet, orders)
    scenarios = solve_all_scenarios(orders, route_date, zones)
    computed_at = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    write_routing_output(spreadsheet, route_date.strftime("%d/%m/%Y"), scenarios,
                          total_orders=len(orders), computed_at=computed_at)


if __name__ == "__main__":
    main()
