"""
Fetches the OSM drive-network graph for your delivery region and saves it
locally as a .graphml file, so solve_routing.py can load it instantly
instead of querying the Overpass API on every single routing run.

Run this via the "Refresh Map Cache" GitHub Actions workflow (manual or on
a schedule) - that workflow commits the resulting file back to the repo
automatically. You can also run it locally if you have internet access:

    pip install osmnx networkx
    python download_map_cache.py

This covers a generous bounding box around your depot, wide enough to
include the Banten / DKI Jakarta / Jawa Barat border area your deliveries
fall in. Widen MAP_CACHE_BBOX below if your delivery area expands beyond
this box in the future - solve_routing.py will fall back to a live Overpass
fetch automatically for any point outside the cached area, so nothing
breaks in the meantime, it just won't benefit from the cache for those
specific runs.
"""
import os
import sys

import osmnx as ox

# Generous box around the depot (-6.2393, 106.5353) covering the
# Tangerang / Jakarta-border delivery area seen in real order data so far.
# (north, south, east, west)
MAP_CACHE_BBOX = (-6.05, -6.45, 106.80, 106.30)

CACHE_DIR = os.path.join(os.path.dirname(__file__), "map_cache")
CACHE_FILE = os.path.join(CACHE_DIR, "region.graphml")


def main():
    north, south, east, west = MAP_CACHE_BBOX
    print(f"Fetching OSM drive network for bbox N{north} S{south} E{east} W{west} ...")
    print("This can take a few minutes depending on Overpass API load.")

    graph = ox.graph_from_bbox((north, south, east, west), network_type="drive")
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
        sys.exit(1)
