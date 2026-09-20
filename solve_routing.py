"""
Daily delivery routing (VRP) solver.

Reads deliverable orders from Order_Outstanding_Cleaned, assigns them to
3 vehicles subject to 9-hour time limits, and writes the plan to Google Sheets. 

Routing Strategy:
1. Protects H+3 SLAs (Tier 1 Critical Priority).
2. Uses PCA math to find the delivery corridor and drop lateral outliers (Far East/West).
3. Keeps Depth (Deep South) to clear distant regions and prevent repeat empty trips.
4. Groups stops into dense Latitudinal Bands (North to South) with zero overlap.
5. Trusts Google OR-Tools Guided Local Search to sequence stops perfectly without overlapping.
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
    for fmt in ("%m/%d/%Y", "%d/%m/%Y", "%Y-%m-%d", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.datetime.strptime(str(value).strip(), fmt).date()
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


TRAFFIC_TIME_MULTIPLIER = _env_num("TRAFFIC_TIME_MULTIPLIER", 1.5)

def _apply_traffic_buffer(time_min):
    n = len(time_min)
    return [[time_min[i][j] * TRAFFIC_TIME_MULTIPLIER for j in range(n)] for i in range(n)]

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

    for idx in range(1, len(locations)):
        age = locations[idx].get("age", 0)
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
        
        # We rely 100% on the optimized OR-Tools sequence here. No custom greedy resequencing.
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                node_order.append(node)
            idx = solution.Value(routing.NextVar(idx))

        seq = box = dist = arr_min = 0
        prev = 0
        v_route = []
        for node in node_order:
            seq += 1; visited.add(node)
            box += locations[node].get("box", 0)
            leg_dist = dist_km[prev][node]
            dist += leg_dist
            
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


# ==========================================
# GEOGRAPHIC CLUSTERING
# ==========================================
def _estimated_max_stops_per_cluster():
    """
    Dynamically calculates the physical ceiling on how many stops a driver 
    can actually hit within the 9-hour shift, based on average travel time.
    """
    avg_travel_min_estimate = 8  
    per_stop_min = SERVICE_TIME_BASE_MINUTES + avg_travel_min_estimate
    return max(5, int(EFFECTIVE_SHIFT_MINUTES / per_stop_min))


def _calculate_lateral_distances(orders):
    """
    Pure Python PCA approximation.
    Finds the main North-South delivery corridor and calculates how far 
    East or West (Lateral Distance) each order is from that main line.
    """
    n = len(orders)
    if n < 2:
        return [0]*n
        
    mean_lat = sum(o['lat'] for o in orders) / n
    mean_lng = sum(o['lng'] for o in orders) / n
    
    cov_ll = sum((o['lat']-mean_lat)**2 for o in orders) / (n - 1) if n > 1 else 0
    cov_gg = sum((o['lng']-mean_lng)**2 for o in orders) / (n - 1) if n > 1 else 0
    cov_lg = sum((o['lat']-mean_lat)*(o['lng']-mean_lng) for o in orders) / (n - 1) if n > 1 else 0
    
    trace = cov_ll + cov_gg
    det = cov_ll * cov_gg - cov_lg**2
    
    # Calculate primary axis
    inner = max(0, trace**2 - 4*det)
    lambda1 = (trace + math.sqrt(inner)) / 2
        
    if cov_lg != 0:
        vec1_lat = lambda1 - cov_gg
        vec1_lng = cov_lg
    else:
        vec1_lat, vec1_lng = 1, 0
        
    mag = math.hypot(vec1_lat, vec1_lng)
    if mag == 0:
        vec1_lat, vec1_lng = 1, 0
    else:
        vec1_lat /= mag
        vec1_lng /= mag
    
    # Normal vector (perpendicular to the corridor)
    norm_lat = -vec1_lng
    norm_lng = vec1_lat
    
    lateral_dists = []
    for o in orders:
        dx = o['lat'] - mean_lat
        dy = o['lng'] - mean_lng
        lateral_dists.append(abs(dx * norm_lat + dy * norm_lng))
        
    return lateral_dists


def cluster_smart_corridor_sweep(orders, n_clusters, capacity_box):
    """
    Smart Corridor Strategy:
    1. Critical Age (3+ days) are routed unconditionally.
    2. Lateral Outliers (Far East/West) are dropped to the backlog.
    3. Farthest Depth (Deep South) is prioritized to clear the edges.
    4. Closest points to the depot are saved for tomorrow's backlog.
    """
    if not orders or n_clusters <= 0:
        return [[] for _ in range(n_clusters)], list(orders)

    lateral_dists = _calculate_lateral_distances(orders)
    
    for i, o in enumerate(orders):
        o['lateral_dist'] = lateral_dists[i]
        o['dist'] = haversine_km(o['lat'], o['lng'], DEPOT_LAT, DEPOT_LNG)
        o['critical'] = 1 if o.get('age', 0) >= 3 else 0
        # Flag orders that stray too far East/West off the main corridor
        o['is_lateral_outlier'] = 1 if o['lateral_dist'] > 0.05 else 0
        
    # The Ultimate Sorting Logic:
    # 1. Critical orders first (-x['critical'])
    # 2. Push lateral outliers to the bottom (x['is_lateral_outlier'])
    # 3. Route the Farthest points first to clear the deep south (-x['dist'])
    orders_sorted_by_priority = sorted(orders, key=lambda x: (
        -x['critical'], 
        x['is_lateral_outlier'], 
        -x['dist']
    ))
    
    max_stops = _estimated_max_stops_per_cluster()
    total_allowed_stops = n_clusters * max_stops
    
    selected_orders = []
    unassigned = []
    current_fleet_boxes = 0
    
    for o in orders_sorted_by_priority:
        fits_stop_count = len(selected_orders) < total_allowed_stops
        fits_box_count = (current_fleet_boxes + o.get('box', 0)) <= (n_clusters * capacity_box)
        
        if fits_stop_count and fits_box_count:
            selected_orders.append(o)
            current_fleet_boxes += o.get('box', 0)
        else:
            unassigned.append(o)
            
    # Spatial Sweep: North to South horizontal bands
    selected_orders_by_lat = sorted(selected_orders, key=lambda x: x['lat'], reverse=True)
    
    clusters = [[] for _ in range(n_clusters)]
    cluster_loads = [0.0] * n_clusters
    
    current_cluster = 0
    for order in selected_orders_by_lat:
        if (len(clusters[current_cluster]) >= max_stops) or \
           (cluster_loads[current_cluster] + order.get('box', 0) > capacity_box):
            
            if current_cluster + 1 < n_clusters:
                current_cluster += 1
            else:
                unassigned.append(order)
                continue
                
        clusters[current_cluster].append(order)
        cluster_loads[current_cluster] += order.get('box', 0)
            
    return clusters, unassigned


def solve_clustered_scenario(orders, route_date, n, depot_lat, depot_lng):
    # Dynamically assign horizontal bands while dropping lateral outliers
    clusters, unassigned = cluster_smart_corridor_sweep(orders, n, VEHICLE_CAPACITY_BOX)
    unassigned_reason = "Lateral Outlier / Time Limit Cutoff"

    all_routes, all_backlog, all_summary = [], [], []

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


def solve_all_scenarios(orders, route_date):
    scenarios = {}
    for n in range(1, NUM_VEHICLES + 1):
        print(f"\n--- Scenario: {n} vehicle(s) ---")
        try:
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

    for o in orders:
        order_date = parse_date_flexible(o.get("tanggal_masuk"))
        o["age"] = max(0, (route_date - order_date).days) if order_date else 0

    scenarios = solve_all_scenarios(orders, route_date)
    computed_at = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    write_routing_output(spreadsheet, route_date.strftime("%d/%m/%Y"), scenarios,
                          total_orders=len(orders), computed_at=computed_at)

if __name__ == "__main__":
    main()
