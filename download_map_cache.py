"""
Fetches the OSM drive-network graph for your delivery region and saves it
locally as a .graphml file, so solve_routing.py can load it instantly
instead of querying the Overpass API on every single routing run.

Run this via the "Refresh Map Cache" GitHub Actions workflow (manual or on
a schedule) - that workflow commits the resulting file back to the repo
automatically. You can also run it locally if you have internet access:

    pip install osmnx networkx
    python download_map_cache.py

Widen MAP_CACHE_BBOX below if your delivery area expands beyond this box in
the future - solve_routing.py will fall back to a live Overpass fetch
automatically for any point outside the cached area, so nothing breaks in
the meantime, it just won't benefit from the cache for those specific runs.
"""
import os
import sys
import time

import osmnx as ox

# Tight-ish box around the depot (-6.2389, 106.5354), sized to the delivery
# area actually seen in real order data so far (Tangerang/Serpong/Cisauk),
# with padding for growth - not the full Jakarta metro area, which is both
# unnecessary and much more likely to time out against Overpass.
# (north, south, east, west)
MAP_CACHE_BBOX = (-6.05, -6.45, 106.72, 106.30)

# Public Overpass mirrors, tried in order. overpass-api.de (the default) is
# frequently overloaded for larger queries; the others are usually more
# reliable for a box this size. All requests still pull the SAME underlying
# OpenStreetMap data - these are just different servers answering it.
OVERPASS_MIRRORS = [
    "https://overpass-api.de/api",
    "https://overpass.kumi.systems/api",
    "https://overpass.openstreetmap.ru/api",
    "https://overpass.private.coffee/api",
]

TIMEOUT_SECONDS = 300  # per attempt

CACHE_DIR = os.path.join(os.path.dirname(__file__), "map_cache")
CACHE_FILE = os.path.join(CACHE_DIR, "region.graphml")


def fetch_graph_with_retries():
    north, south, east, west = MAP_CACHE_BBOX
    last_error = None

    for i, mirror in enumerate(OVERPASS_MIRRORS, start=1):
        print(f"[{i}/{len(OVERPASS_MIRRORS)}] trying Overpass mirror: {mirror}")
        ox.settings.overpass_endpoint = mirror
        ox.settings.timeout = TIMEOUT_SECONDS
        try:
            graph = ox.graph_from_bbox((north, south, east, west), network_type="drive")
            print(f"  succeeded via {mirror}")
            return graph
        except Exception as e:
            print(f"  failed via {mirror}: {e}")
            last_error = e
            if i < len(OVERPASS_MIRRORS):
                time.sleep(5)

    raise RuntimeError(f"All {len(OVERPASS_MIRRORS)} Overpass mirrors failed. Last error: {last_error}")


def main():
    north, south, east, west = MAP_CACHE_BBOX
    print(f"Fetching OSM drive network for bbox N{north} S{south} E{east} W{west} ...")
    print(f"Box size: ~{abs(north - south) * 111:.0f}km x ~{abs(east - west) * 111:.0f}km")
    print("This can take a few minutes depending on Overpass API load.")

    graph = fetch_graph_with_retries()
    graph = ox.add_edge_speeds(graph)
    graph = ox.add_edge_travel_times(graph)

    print(f"Graph fetched: {len(graph.nodes)} nodes, {len(graph.edges)} edges")

    os.makedirs(CACHE_DIR, exist_ok=True)
    ox.save_graphml(graph, CACHE_FILE)

    size_mb = os.path.getsize(CACHE_FILE) / (1024 * 1024)
    print(f"Saved to {CACHE_FILE} ({size_mb:.1f} MB)")

    if size_mb > 50:
        print("NOTE: this file is over 50MB - GitHub will warn on push. "
              "Consider Git LFS if it keeps growing, or shrink MAP_CACHE_BBOX.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"FATAL: could not fetch/save map cache: {e}")
        print("This is fine short-term - solve_routing.py falls back to live "
              "per-run Overpass fetches (much smaller queries) if no cache "
              "exists yet. Re-run this workflow later, or try again in a "
              "few hours if Overpass is having a bad day across all mirrors.")
        sys.exit(1)
