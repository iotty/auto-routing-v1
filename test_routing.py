"""
Local verification harness - NOT part of the GitHub Actions pipeline.
Generates synthetic orders (no Google Sheets needed) and checks that
solve_all_scenarios() respects every hard constraint across all three
fleet-size scenarios, and that priority-based backlogging works.
"""
import random
import datetime
import sys
import time

sys.path.insert(0, ".")
from solve_routing import (
    solve_all_scenarios, VEHICLE_CAPACITY_BOX,
    SHIFT_LENGTH_MINUTES, NUM_VEHICLES, DEPOT_LAT, DEPOT_LNG,
)

random.seed(42)

ROUTE_DATE = datetime.date(2026, 9, 14)  # arbitrary "H+1" for the test

# Tangerang-ish bounding box, matching the real coordinates we've seen.
LAT_RANGE = (-6.35, -6.15)
LNG_RANGE = (106.40, 106.70)


def make_synthetic_orders(n, min_box=1, max_box=10, max_age_days=6):
    orders = []
    for i in range(n):
        age = random.randint(0, max_age_days)
        order_date = ROUTE_DATE - datetime.timedelta(days=age)
        orders.append({
            "kode_fulfillment": f"TEST{i:03d}",
            "nama_penerima": f"Test Outlet {i}",
            "alamat_penerima": f"Jl. Test No. {i}",
            "lat": random.uniform(*LAT_RANGE),
            "lng": random.uniform(*LNG_RANGE),
            "box": random.randint(min_box, max_box),
            "tanggal_masuk": order_date.strftime("%d/%m/%Y"),
            "_age_days": age,  # test-only field for verification
        })
    return orders


def run_case(label, n_orders, **kwargs):
    print(f"\n=== {label}: {n_orders} synthetic orders ===")
    orders = make_synthetic_orders(n_orders, **kwargs)
    order_by_id = {o["kode_fulfillment"]: o for o in orders}

    t0 = time.time()
    scenarios = solve_all_scenarios(orders, ROUTE_DATE)
    elapsed = time.time() - t0
    print(f"All {len(scenarios)} scenarios solved in {elapsed:.1f}s total")

    ok = True

    for n, result in scenarios.items():
        routes, backlog, summary = result["routes"], result["backlog"], result["summary"]
        served_ids = {r["kode_fulfillment"] for r in routes}
        backlog_ids = {b["kode_fulfillment"] for b in backlog}

        if len(served_ids) + len(backlog_ids) != n_orders:
            print(f"FAIL [{n} trucks]: served+backlogged ({len(served_ids)}+{len(backlog_ids)}) != total ({n_orders})")
            ok = False
        if not served_ids.isdisjoint(backlog_ids):
            print(f"FAIL [{n} trucks]: some order both served and backlogged")
            ok = False

        for row in summary:
            vehicle, driver, stops, box, dist_km, duration_min, pct, flag = row
            if box > VEHICLE_CAPACITY_BOX:
                print(f"FAIL [{n} trucks]: {vehicle} over capacity: {box} > {VEHICLE_CAPACITY_BOX}")
                ok = False
            if duration_min > SHIFT_LENGTH_MINUTES:
                print(f"FAIL [{n} trucks]: {vehicle} over shift length: {duration_min} > {SHIFT_LENGTH_MINUTES}")
                ok = False

        # More trucks available should never serve FEWER orders overall
        # (monotonicity check - a sign something's wrong if not).
        total_served_this_scenario = len(served_ids)
        print(f"  [{n} truck(s) available] served={total_served_this_scenario}, "
              f"backlog={len(backlog_ids)}")
        for row in summary:
            vehicle, driver, stops, box, dist_km, duration_min, pct, flag = row
            note = f" <- {flag}" if flag else ""
            print(f"    {vehicle} ({driver}): {stops} stops, {box} box, {dist_km} km, {duration_min} min, {pct}%{note}")

        if backlog_ids:
            avg_age_served = sum(order_by_id[i]["_age_days"] for i in served_ids) / max(len(served_ids), 1)
            avg_age_backlog = sum(order_by_id[i]["_age_days"] for i in backlog_ids) / len(backlog_ids)
            if avg_age_served < avg_age_backlog:
                print(f"FAIL [{n} trucks]: served orders NEWER on average ({avg_age_served:.2f}) "
                      f"than backlogged ({avg_age_backlog:.2f}) - priority logic backwards!")
                ok = False

    served_counts = [len(scenarios[n]["routes"]) for n in sorted(scenarios)]
    if served_counts != sorted(served_counts):
        print(f"FAIL: serving fewer orders with MORE trucks available is suspicious: {served_counts}")
        ok = False
    else:
        print(f"OK: served count is non-decreasing as fleet size grows: {served_counts}")

    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    results = []

    results.append(run_case("Very light day", 15))
    results.append(run_case("Realistic current volume (~36 orders)", 36))
    results.append(run_case("Heavier day", 70))

    # Tight-cluster check: with all 3 vehicles available but orders clustered
    # close together, a single vehicle should be able to comfortably take a
    # generous number of stops - not capped at any arbitrary number, purely
    # bounded by real capacity/time.
    print("\n=== Tight cluster check (no arbitrary stop cap) ===")
    tight_orders = make_synthetic_orders(40, min_box=1, max_box=3)
    for o in tight_orders:
        o["lat"] = DEPOT_LAT + random.uniform(-0.03, 0.03)
        o["lng"] = DEPOT_LNG + random.uniform(-0.03, 0.03)
    scenarios = solve_all_scenarios(tight_orders, ROUTE_DATE)
    max_stops_1truck = max((row[2] for row in scenarios[1]["summary"]), default=0)
    print(f"With 1 truck available, tight cluster: {max_stops_1truck} stops on that truck")
    results.append(True)  # informational

    print("\n=== OVERALL ===")
    if all(results):
        print("ALL CASES PASSED")
        sys.exit(0)
    else:
        print("SOME CASES FAILED")
        sys.exit(1)
