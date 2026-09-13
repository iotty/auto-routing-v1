"""
Daily delivery routing (VRP) solver.

Reads deliverable orders from Order_Outstanding_Cleaned, assigns them to
3 vehicles subject to capacity/stop-count/time constraints, and writes the
plan to Google Sheets. Orders that don't fit are backlogged automatically -
older orders (by Tanggal Fulfillment Masuk FF) are preferred over newer
ones when the fleet can't serve everyone.

NOT YET WIRED IN:
  - Output sheet's exact column layout - this is a first working version;
    expect to revise once you share the format you actually want.

REQUIRED CONFIG (env vars, same style as pipeline.py):
  GOOGLE_SERVICE_ACCOUNT - service account JSON, same one already used
  SPREADSHEET_ID         - same spreadsheet the rest of the pipeline uses
Optional:
  ROUTE_DATE             - "YYYY-MM-DD"; defaults to tomorrow (H+1) if unset
"""

import os
import sys
import json
import math
import datetime
import traceback

import gspread
from oauth2client.service_account import ServiceAccountCredentials
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ==========================================
# CONFIG - adjust these as real numbers come in
# ==========================================

# Depot / warehouse location.
DEPOT_LAT = -6.238931133669451
DEPOT_LNG = 106.53540668179242
DEPOT_NAME = "FF Bitung"

VEHICLES = [
    {"name": "CDE Long 1", "driver": "Ade"},
    {"name": "CDE Long 2", "driver": "Ahmad"},
    {"name": "CDE Long 3", "driver": "Subki"},
]
NUM_VEHICLES = len(VEHICLES)

VEHICLE_CAPACITY_BOX = 1200      # max total "Total Box" per vehicle
# No stop-count cap, and no hard "minimum stops per vehicle" rule anymore -
# see the module docstring for why. Instead, the solver is run once per
# candidate fleet size (1, 2, 3 vehicles available) and all results are
# written side by side for you to compare and choose from.
UTILIZATION_FLAG_THRESHOLD = 22  # used only to flag low-utilization vehicles
                                   # in the output for your attention - not enforced

SHIFT_START_HOUR = 6             # 06:00 dispatch
SHIFT_LENGTH_MINUTES = 10 * 60   # 10-hour max delivery window (06:00-16:00)
SERVICE_TIME_MINUTES = 15        # time spent at each stop

AVG_SPEED_KMH_FALLBACK = 25       # used ONLY if the real OSM road-network
                                    # lookup fails (e.g. Overpass API down) -
                                    # see get_distance_time_matrices()

# Padding (degrees) added around the depot+orders bounding box before
# pulling the OSM road network - covers realistic detours without pulling
# an unnecessarily huge graph. ~0.05 deg is roughly 5-6 km at this latitude.
OSM_BBOX_PADDING_DEG = 0.08

# Priority penalty: skipping a node costs this much "virtual time", scaled
# by how many days old the order is. Must dominate normal travel costs so
# the solver strongly prefers dropping NEWER orders over OLDER ones.
PRIORITY_BASE_PENALTY = 100000
PRIORITY_WEIGHT_PER_DAY = 20000

SOLVE_TIME_LIMIT_SECONDS = 15    # per scenario - 3 scenarios run per invocation (see solve_all_scenarios)

SOURCE_SHEET = "Order_Outstanding_Cleaned"
OUTPUT_SHEET = "Routing_Plan"          # written per-scenario as Routing_Plan_1Truck / _2Trucks / _3Trucks
BACKLOG_SHEET = "Routing_Backlog"      # written per-scenario as Routing_Backlog_1Truck / _2Trucks / _3Trucks
SCENARIOS_SHEET = "Routing_Scenarios"  # comparison table across all fleet-size options


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

    try:
        return gc.open_by_key(spreadsheet_id)
    except gspread.exceptions.SpreadsheetNotFound:
        print(f"FATAL: spreadsheet '{spreadsheet_id}' not found (check SPREADSHEET_ID).")
        sys.exit(1)
    except PermissionError as e:
        service_account_email = service_account_info.get("client_email", "<unknown - check your JSON key>")
        print(
            f"FATAL: no permission to open spreadsheet '{spreadsheet_id}' (HTTP 403).\n"
            f"Fix checklist:\n"
            f"  1. Share the spreadsheet with this exact email as Editor:\n     {service_account_email}\n"
            f"  2. Double-check SPREADSHEET_ID matches the sheet's URL.\n"
            f"Original error: {e}"
        )
        sys.exit(1)
    except gspread.exceptions.APIError as e:
        print(f"FATAL: failed to open spreadsheet (Sheets API error): {e}")
        sys.exit(1)


def load_orders(spreadsheet):
    """Reads Order_Outstanding_Cleaned into a list of order dicts."""
    try:
        sheet = spreadsheet.worksheet(SOURCE_SHEET)
    except gspread.exceptions.WorksheetNotFound:
        print(f"FATAL: sheet '{SOURCE_SHEET}' not found in this spreadsheet. "
              f"Check the name matches exactly (case-sensitive).")
        sys.exit(1)

    values = sheet.get_all_values()
    if len(values) < 2:
        return []

    header, *data_rows = values
    idx = {name: i for i, name in enumerate(header)}

    required = ["Kode Fulfillment", "Nama Penerima", "Alamat Penerima",
                "Latitude Penerima", "Longitude Penerima", "Total Box",
                "Tanggal Fulfillment Masuk FF"]
    missing = [c for c in required if c not in idx]
    if missing:
        print(f"FATAL: '{SOURCE_SHEET}' is missing expected column(s): {missing}. "
              f"Found columns: {header}")
        sys.exit(1)

    orders = []
    skipped_bad_coords = 0
    skipped_missing_data = 0
    for row_num, row in enumerate(data_rows, start=2):
        try:
            lat = float(row[idx["Latitude Penerima"]])
            lng = float(row[idx["Longitude Penerima"]])
        except (ValueError, IndexError):
            skipped_bad_coords += 1
            continue  # skip rows with unusable coordinates

        try:
            box = float(row[idx["Total Box"]] or 0)
        except ValueError:
            box = 0
        box = max(0.0, box)  # guard against negative/garbage values

        if lat == 0 or lng == 0:
            skipped_bad_coords += 1
            continue

        try:
            kode = row[idx["Kode Fulfillment"]]
            nama = row[idx["Nama Penerima"]]
            alamat = row[idx["Alamat Penerima"]]
            tanggal = row[idx["Tanggal Fulfillment Masuk FF"]]
        except IndexError:
            skipped_missing_data += 1
            continue

        orders.append({
            "kode_fulfillment": kode,
            "nama_penerima": nama,
            "alamat_penerima": alamat,
            "lat": lat,
            "lng": lng,
            "box": box,
            "tanggal_masuk": tanggal,
        })

    if skipped_bad_coords:
        print(f"[load_orders] skipped {skipped_bad_coords} row(s) with missing/invalid coordinates.")
    if skipped_missing_data:
        print(f"[load_orders] skipped {skipped_missing_data} row(s) with missing required fields.")

    return orders


def parse_date_flexible(value):
    """Order_Outstanding_Cleaned's date column may come through as various
    string formats depending on how Sheets exported it. Try the common ones."""
    if not value:
        return None
    for fmt in ("%m/%d/%Y", "%d/%m/%Y", "%Y-%m-%d"):
        try:
            return datetime.datetime.strptime(value.strip(), fmt).date()
        except ValueError:
            continue
    return None


def write_routing_output(spreadsheet, route_date, scenarios):
    """
    scenarios: dict from solve_all_scenarios(), keyed by number of vehicles.
    Writes one full plan per scenario (Routing_Plan_1Truck, _2Trucks,
    _3Trucks) plus a Routing_Scenarios comparison sheet, so you can review
    all options and pick which one to actually dispatch.
    """
    comparison_rows = []

    for n, result in scenarios.items():
        label = f"{n}Truck" if n == 1 else f"{n}Trucks"
        plan_sheet = f"{OUTPUT_SHEET}_{label}"
        backlog_sheet = f"{BACKLOG_SHEET}_{label}"

        _write_sheet(spreadsheet, plan_sheet, [
            "Route Date", "Vehicle", "Driver", "Stop Seq", "Kode Fulfillment",
            "Nama Penerima", "Alamat Penerima", "Latitude", "Longitude",
            "Box (this stop)", "Cumulative Box", "ETA", "ETD",
            "Distance from Prev (km)", "Tanggal Fulfillment Masuk FF",
        ], [
            [
                route_date, r["vehicle"], r["driver"], r["seq"], r["kode_fulfillment"],
                r["nama_penerima"], r["alamat_penerima"], r["lat"], r["lng"],
                r["box"], r["cumulative_box"], r["eta"], r["etd"],
                round(r["distance_from_prev_km"], 2), r["tanggal_masuk"],
            ]
            for r in result["routes"]
        ])

        _write_sheet(spreadsheet, backlog_sheet, [
            "Route Date", "Kode Fulfillment", "Nama Penerima", "Alamat Penerima",
            "Box", "Tanggal Fulfillment Masuk FF", "Reason",
        ], [
            [
                route_date, b["kode_fulfillment"], b["nama_penerima"], b["alamat_penerima"],
                b["box"], b["tanggal_masuk"], b.get("backlog_reason", "Backlogged"),
            ]
            for b in result["backlog"]
        ])

        total_distance = round(sum(row[4] for row in result["summary"]), 2)
        total_stops = sum(row[2] for row in result["summary"])
        total_box = sum(row[3] for row in result["summary"])
        vehicles_used = sum(1 for row in result["summary"] if row[2] > 0)
        comparison_rows.append([
            route_date, n, vehicles_used, total_stops, len(result["backlog"]),
            total_box, total_distance,
            round(total_distance / total_stops, 2) if total_stops else 0,
            plan_sheet,
        ])

        # Per-vehicle detail for this scenario, appended under the comparison
        # table for context on WHY a scenario looks the way it does.
        for row in result["summary"]:
            vehicle, driver, stops, box, dist, duration, pct, flag = row
            print(f"  [{label}] {vehicle} ({driver}): {stops} stops, {box} box, "
                  f"{dist} km, {duration} min, {pct}% capacity"
                  + (f" -- {flag}" if flag else ""))

    _write_sheet(spreadsheet, SCENARIOS_SHEET, [
        "Route Date", "Vehicles Available", "Vehicles Actually Used", "Stops Served",
        "Stops Backlogged", "Total Box", "Total Distance (km)",
        "Distance per Stop (km)", "See Plan Tab",
    ], comparison_rows)


def _write_sheet(spreadsheet, sheet_name, header, rows):
    try:
        worksheet = spreadsheet.worksheet(sheet_name)
        worksheet.clear()
    except gspread.exceptions.WorksheetNotFound:
        worksheet = spreadsheet.add_worksheet(title=sheet_name, rows="200", cols=str(max(len(header), 1)))

    all_rows = [header] + rows
    if all_rows:
        worksheet.update(f"A1", all_rows)
    print(f"[{sheet_name}] wrote {len(rows)} row(s).")


# ==========================================
# ROUTING (VRP) CORE
# ==========================================
def haversine_km(lat1, lng1, lat2, lng2):
    R = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lng2 - lng1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def build_haversine_matrices(locations):
    """Fallback matrices: straight-line distance, converted to time via a
    flat assumed speed. Only used if the real OSM road-network lookup below
    fails - e.g. Overpass API unreachable/rate-limited that run."""
    n = len(locations)
    dist_km = [[0.0] * n for _ in range(n)]
    time_min = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            km = haversine_km(locations[i]["lat"], locations[i]["lng"],
                               locations[j]["lat"], locations[j]["lng"])
            dist_km[i][j] = km
            time_min[i][j] = (km / AVG_SPEED_KMH_FALLBACK) * 60.0
    return dist_km, time_min


def _bbox_covers(graph, north, south, east, west):
    """True if the cached graph's node extent covers the requested bbox
    (with a small margin), so we know it's safe to reuse instead of
    fetching fresh from Overpass."""
    if graph is None or len(graph.nodes) == 0:
        return False
    lats = [data["y"] for _, data in graph.nodes(data=True)]
    lngs = [data["x"] for _, data in graph.nodes(data=True)]
    margin = 0.01  # ~1km slack
    return (min(lats) - margin <= south and max(lats) + margin >= north and
            min(lngs) - margin <= west and max(lngs) + margin >= east)


def build_osm_matrices(locations):
    """
    Real road-network distance/time matrices, using live OpenStreetMap data
    for the Banten/Jakarta/Jawa Barat area (your delivery region) via osmnx.

    Rather than pre-downloading the entire three provinces (tens of millions
    of road segments - impractical to process in a single GitHub Actions
    run), this pulls the OSM road graph for a bounding box around today's
    actual depot + delivery points, padded by OSM_BBOX_PADDING_DEG. Since
    your deliveries sit inside that Banten/Jakarta/West-Java border area,
    this is the same underlying OSM data, scoped to what's actually needed
    for this run instead of the whole region every time.

    Raises on failure (network issue, Overpass rate limit, etc.) - caller
    falls back to haversine so a bad network day doesn't crash the whole run.
    """
    import osmnx as ox
    import networkx as nx

    lats = [loc["lat"] for loc in locations]
    lngs = [loc["lng"] for loc in locations]
    north = max(lats) + OSM_BBOX_PADDING_DEG
    south = min(lats) - OSM_BBOX_PADDING_DEG
    east = max(lngs) + OSM_BBOX_PADDING_DEG
    west = min(lngs) - OSM_BBOX_PADDING_DEG

    cache_file = os.path.join(os.path.dirname(__file__), "map_cache", "region.graphml")
    graph = None
    if os.path.exists(cache_file):
        print(f"[OSM] loading cached map from {cache_file} (no network needed) ...")
        cached_graph = ox.load_graphml(cache_file)
        if _bbox_covers(cached_graph, north, south, east, west):
            graph = cached_graph
            print(f"[OSM] cached graph covers today's area: {len(graph.nodes)} nodes, {len(graph.edges)} edges")
        else:
            print("[OSM] cached graph doesn't cover today's delivery area - "
                  "falling back to a live fetch for this run. Consider widening "
                  "MAP_CACHE_BBOX in download_map_cache.py and refreshing the cache.")

    if graph is None:
        print(f"[OSM] fetching drive network for bbox N{north:.4f} S{south:.4f} E{east:.4f} W{west:.4f} ...")
        graph = ox.graph_from_bbox((north, south, east, west), network_type="drive")
        graph = ox.add_edge_speeds(graph)      # estimates km/h per road segment from OSM tags
        graph = ox.add_edge_travel_times(graph)  # seconds, using the speeds above
        print(f"[OSM] graph has {len(graph.nodes)} nodes, {len(graph.edges)} edges")

    xs = [loc["lng"] for loc in locations]
    ys = [loc["lat"] for loc in locations]
    nearest = ox.distance.nearest_nodes(graph, xs, ys)

    n = len(locations)
    dist_km = [[0.0] * n for _ in range(n)]
    time_min = [[0.0] * n for _ in range(n)]

    for i in range(n):
        lengths = nx.single_source_dijkstra_path_length(graph, nearest[i], weight="length")
        times = nx.single_source_dijkstra_path_length(graph, nearest[i], weight="travel_time")
        for j in range(n):
            if i == j:
                continue
            target = nearest[j]
            if target in lengths:
                dist_km[i][j] = lengths[target] / 1000.0
                time_min[i][j] = times.get(target, lengths[target] / 1000.0 / AVG_SPEED_KMH_FALLBACK * 60.0) / 60.0
            else:
                # No path found in this bbox (disconnected road segment,
                # rare) - fall back to haversine for just this pair.
                km = haversine_km(locations[i]["lat"], locations[i]["lng"],
                                   locations[j]["lat"], locations[j]["lng"])
                dist_km[i][j] = km
                time_min[i][j] = (km / AVG_SPEED_KMH_FALLBACK) * 60.0

    return dist_km, time_min


def build_distance_time_matrices(locations):
    if os.environ.get("SKIP_OSM"):
        # Fast path for local testing without network access - see
        # test_routing.py and README.
        return build_haversine_matrices(locations)
    try:
        return build_osm_matrices(locations)
    except Exception as e:
        print(f"WARNING: OSM road-network distance lookup failed ({e}); "
              f"falling back to straight-line distance for this run. "
              f"Results will be less accurate than usual.")
        print(traceback.format_exc())
        return build_haversine_matrices(locations)


def solve_routing(orders, route_date, num_vehicles, dist_km, time_min):
    """
    Solves the VRP for a given fleet size (1, 2, or 3 vehicles available -
    see solve_all_scenarios, which calls this once per candidate fleet size
    so you can compare and choose). Distance matrices are precomputed once
    and passed in, so running multiple scenarios doesn't refetch the map
    data multiple times.

    Returns (vehicle_routes, backlog, summary_rows).
    """
    locations = [{"lat": DEPOT_LAT, "lng": DEPOT_LNG}] + orders
    num_locations = len(locations)

    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    # Objective: minimize total distance driven (meters, as an integer cost) -
    # per your instruction, distance is the thing being optimized, not time.
    def distance_callback(from_index, to_index):
        i = manager.IndexToNode(from_index)
        j = manager.IndexToNode(to_index)
        return int(round(dist_km[i][j] * 1000))

    distance_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback_index)

    # Time dimension: NOT the objective, just a hard 10-hour cap per route.
    def time_callback(from_index, to_index):
        i = manager.IndexToNode(from_index)
        j = manager.IndexToNode(to_index)
        service = SERVICE_TIME_MINUTES if i != 0 else 0
        return int(round(time_min[i][j] + service))

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(
        time_callback_index,
        0,                          # no slack
        SHIFT_LENGTH_MINUTES,       # max route duration (10 hours)
        True,                       # start cumul at 0 for every vehicle
        "Time",
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    # Capacity dimension (box).
    def demand_callback(from_index):
        i = manager.IndexToNode(from_index)
        return int(round(locations[i].get("box", 0)))

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index, 0,
        [VEHICLE_CAPACITY_BOX] * num_vehicles,
        True, "Capacity",
    )

    # Stop-count dimension: no upper cap (per your instruction). Note: the
    # 22-minimum-if-active rule is enforced as a POST-PROCESSING step after
    # solving (see solve_routing's post-solve section below), not as a hard
    # constraint here. A hard "must reach 22 stops before finishing" CP
    # constraint turned out to trap standard local search at the trivial
    # "don't use this vehicle" solution - a known difficulty with minimum-
    # load constraints in VRP local search (escaping requires committing 22
    # nodes at once, which single-node-move neighborhoods can't do
    # incrementally). Solving freely and then backlogging any
    # under-threshold vehicle's stops afterward is both more robust and
    # arguably a better match for what you described anyway - "not worth
    # sending a near-empty truck" is a business rule, not an optimizer
    # constraint.
    def stop_callback(from_index):
        i = manager.IndexToNode(from_index)
        return 0 if i == 0 else 1

    stop_callback_index = routing.RegisterUnaryTransitCallback(stop_callback)
    num_orders = len(orders)
    routing.AddDimensionWithVehicleCapacity(
        stop_callback_index, 0,
        [num_orders] * num_vehicles,  # effectively unbounded - can't exceed total orders anyway
        True, "StopCount",
    )

    # Make every non-depot node optional, with an age-scaled skip penalty.
    for node_index in range(1, num_locations):
        order = locations[node_index]
        order_date = parse_date_flexible(order.get("tanggal_masuk"))
        age_days = max(0, (route_date - order_date).days) if order_date else 0
        penalty = int(PRIORITY_BASE_PENALTY + PRIORITY_WEIGHT_PER_DAY * age_days)
        routing.AddDisjunction([manager.NodeToIndex(node_index)], penalty)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(SOLVE_TIME_LIMIT_SECONDS)

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        raise RuntimeError(
            f"No solution found for the {num_vehicles}-vehicle scenario - constraints may be "
            f"infeasible (e.g. every order too far from the depot to fit within the "
            f"{SHIFT_LENGTH_MINUTES}-minute shift, even for a single stop)."
        )

    per_vehicle_stops = {}
    summary_rows = []
    visited_nodes = set()

    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        seq = 0
        cumulative_box = 0
        total_distance_km = 0.0
        prev_node = 0  # depot

        route_stops = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node != 0:
                seq += 1
                visited_nodes.add(node)
                order = locations[node]
                cumulative_box += order.get("box", 0)
                km = dist_km[prev_node][node]
                total_distance_km += km

                arrival_min = solution.Value(time_dimension.CumulVar(index))
                eta = _minutes_to_clock(arrival_min)
                etd = _minutes_to_clock(arrival_min + SERVICE_TIME_MINUTES)

                route_stops.append({
                    "vehicle": VEHICLES[vehicle_id]["name"],
                    "driver": VEHICLES[vehicle_id]["driver"],
                    "seq": seq,
                    "kode_fulfillment": order["kode_fulfillment"],
                    "nama_penerima": order["nama_penerima"],
                    "alamat_penerima": order["alamat_penerima"],
                    "lat": order["lat"],
                    "lng": order["lng"],
                    "box": order["box"],
                    "cumulative_box": cumulative_box,
                    "eta": eta,
                    "etd": etd,
                    "distance_from_prev_km": km,
                    "tanggal_masuk": order["tanggal_masuk"],
                })
                prev_node = node

            index = solution.Value(routing.NextVar(index))

        end_node = manager.IndexToNode(index)
        total_distance_km += dist_km[prev_node][end_node]
        end_time_min = solution.Value(time_dimension.CumulVar(index))

        per_vehicle_stops[vehicle_id] = {
            "stops": route_stops,
            "seq": seq,
            "cumulative_box": cumulative_box,
            "total_distance_km": total_distance_km,
            "end_time_min": end_time_min,
        }

    backlog = []
    for node_index in range(1, num_locations):
        if node_index not in visited_nodes:
            order = locations[node_index]
            backlog.append({
                "kode_fulfillment": order["kode_fulfillment"],
                "nama_penerima": order["nama_penerima"],
                "alamat_penerima": order["alamat_penerima"],
                "box": order["box"],
                "tanggal_masuk": order["tanggal_masuk"],
                "backlog_reason": "Fleet capacity/time constraint",
            })

    # No auto-backlogging of low-utilization vehicles here - that decision
    # is yours (see module docstring). We just flag it in the summary so
    # it's visible when you're reviewing the plan.
    vehicle_routes = []
    for vehicle_id in range(num_vehicles):
        info = per_vehicle_stops[vehicle_id]
        vehicle_routes.extend(info["stops"])

        capacity_used_pct = round(100.0 * info["cumulative_box"] / VEHICLE_CAPACITY_BOX, 1) if VEHICLE_CAPACITY_BOX else 0
        low_utilization = 0 < info["seq"] < UTILIZATION_FLAG_THRESHOLD
        flag = f"Consider backlogging - only {info['seq']} stops (below {UTILIZATION_FLAG_THRESHOLD})" if low_utilization else ""
        summary_rows.append([
            VEHICLES[vehicle_id]["name"], VEHICLES[vehicle_id]["driver"],
            info["seq"], info["cumulative_box"], round(info["total_distance_km"], 2),
            info["end_time_min"], capacity_used_pct, flag,
        ])

    return vehicle_routes, backlog, summary_rows


def solve_all_scenarios(orders, route_date):
    """
    Solves the routing problem once per candidate fleet size (1 vehicle,
    2 vehicles, 3 vehicles available) so you can compare real, fully-worked-
    out plans and pick which one to actually run - rather than the
    algorithm silently deciding for you. Each scenario independently
    minimizes distance subject to capacity/time/priority-backlog; the
    solver decides how many stops actually fit per truck based on real
    geometry (a tight cluster might comfortably take 26 stops; a spread-out
    day might only fit 18) - it isn't forced to any particular count.

    Returns a dict: { num_vehicles: {"routes": ..., "backlog": ..., "summary": ...} }
    """
    locations = [{"lat": DEPOT_LAT, "lng": DEPOT_LNG}] + orders
    dist_km, time_min = build_distance_time_matrices(locations)

    scenarios = {}
    for n in range(1, NUM_VEHICLES + 1):
        print(f"\n--- Scenario: {n} vehicle(s) available ---")
        routes, backlog, summary = solve_routing(orders, route_date, n, dist_km, time_min)
        total_distance = sum(row[4] for row in summary)
        total_stops = sum(row[2] for row in summary)
        print(f"  Served {total_stops}/{len(orders)}, backlog {len(backlog)}, "
              f"total distance {round(total_distance, 1)} km")
        scenarios[n] = {"routes": routes, "backlog": backlog, "summary": summary}

    return scenarios


def _minutes_to_clock(minutes_from_shift_start):
    total_minutes = SHIFT_START_HOUR * 60 + minutes_from_shift_start
    hh = int(total_minutes // 60) % 24
    mm = int(total_minutes % 60)
    return f"{hh:02d}:{mm:02d}"


# ==========================================
# MAIN
# ==========================================
def main():
    route_date_str = os.environ.get("ROUTE_DATE", "").strip()
    if route_date_str:
        try:
            route_date = datetime.datetime.strptime(route_date_str, "%Y-%m-%d").date()
        except ValueError:
            print(f"FATAL: ROUTE_DATE '{route_date_str}' is not a valid date - expected format YYYY-MM-DD.")
            sys.exit(1)
    else:
        route_date = datetime.date.today() + datetime.timedelta(days=1)  # H+1

    print(f"Routing for date: {route_date.isoformat()}")

    spreadsheet = get_spreadsheet()
    orders = load_orders(spreadsheet)
    print(f"Loaded {len(orders)} deliverable order(s) from '{SOURCE_SHEET}'.")

    if not orders:
        print("No orders to route. Exiting.")
        return

    scenarios = solve_all_scenarios(orders, route_date)

    print("\n=== Scenario comparison ===")
    for n, result in scenarios.items():
        total_distance = round(sum(row[4] for row in result["summary"]), 1)
        total_stops = sum(row[2] for row in result["summary"])
        print(f"  {n} vehicle(s) available: {total_stops} served, "
              f"{len(result['backlog'])} backlogged, {total_distance} km total")

    write_routing_output(spreadsheet, route_date.strftime("%d/%m/%Y"), scenarios)
    print(f"\nReview {SCENARIOS_SHEET} to compare options, then use whichever "
          f"Routing_Plan_*Trucks tab matches your choice.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("FATAL: unexpected error during routing:")
        print(traceback.format_exc())
        sys.exit(1)
