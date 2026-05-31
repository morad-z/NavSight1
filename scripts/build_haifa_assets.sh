#!/usr/bin/env bash
# build_haifa_assets.sh — generate the offline Haifa OSM assets (MAP_MATCHING_PLAN.md
# §8M Step A*). Thin wrapper around the Python generator, which is the authoritative,
# no-extra-tools path (fetches Haifa OSM from the Overpass API; no osmium/PBF/JVM).
#
# Output (committed under app/src/main/assets/osm/haifa/):
#   haifa_segments.bin   road-segment R-tree records   (snapping,  Step C*)
#   haifa_graph.bin      routing graph (vertices+edges) (routing,   Step K-routing*)
#   haifa_geocode.bin    place table + token postings   (search,    Step K-search*)
#   haifa_assets.json    manifest (bbox + provenance)
#
# The binary format is pinned by app/src/test/.../OsmDataLayerRoundTripTest.kt and
# validated against the real data by OsmRealAssetsTest.kt.
#
# Prereqs: Python 3 + network. Usage:  bash scripts/build_haifa_assets.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="$(command -v python3 || command -v python)"
exec "${PY}" "${ROOT}/scripts/build_haifa_assets.py" "$@"
