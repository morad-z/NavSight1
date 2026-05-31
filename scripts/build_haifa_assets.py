#!/usr/bin/env python3
"""
build_haifa_assets.py — generate the offline OSM assets for NavSight's
map-matching / routing / search stack (MAP_MATCHING_PLAN.md §8M Step A*).

Two coverage scopes (offline, no key, no osmium/PBF needed — uses Overpass):
  * SEARCH (geocode index)  → ALL OF ISRAEL (254k named places). Cheap to embed,
    so destination search resolves any named place in the country.
  * ROADS  (segments+graph) → a BOUNDED NORTHERN REGION (Haifa + surroundings).
    All-Israel roads (663k ways → ~1.5M segments) would bloat the APK and load
    slowly, so snapping/routing cover the region you actually travel. Widen
    ROADS_BBOX (or run again) if you test elsewhere.

Output (committed under app/src/main/assets/osm/haifa/):
  haifa_segments.bin   road-segment records   (snapping,  Step C*)   [ROADS bbox]
  haifa_graph.bin      routing graph          (routing,   Step K-routing*) [ROADS bbox]
  haifa_geocode.bin    place table + postings (search,    Step K-search*) [SEARCH bbox]
  haifa_assets.json    manifest (both bboxes + provenance)

Binary layout MIRRORS app's OsmAssetFormat.kt; pinned by the round-trip + real-asset tests.
Usage:  python scripts/build_haifa_assets.py
"""
import hashlib
import json
import math
import os
import re
import struct
import sys
import time
import unicodedata
import urllib.request

# (minLon, minLat, maxLon, maxLat)
SEARCH_BBOX = (34.20, 29.45, 35.95, 33.45)   # all of Israel (geocode index) — cheap, search anywhere
ROADS_BBOX  = (34.94, 32.74, 35.10, 32.86)   # Haifa only (snap/route) — all-Israel roads is too big to embed

SCHEMA_VERSION = 1
HIGHWAY_CLASSES = [
    "unknown", "footway", "path", "pedestrian", "steps", "living_street",
    "residential", "unclassified", "tertiary", "secondary", "primary",
    "service", "cycleway", "track",
]
HIGHWAY_CODE = {name: i for i, name in enumerate(HIGHWAY_CLASSES)}

OVERPASS_MIRRORS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "app", "src", "main", "assets", "osm", "haifa")

# ── little-endian packers ──────────────────────────────────────────────────────
def i32(v): return struct.pack("<i", v)
def i64(v): return struct.pack("<q", v)
def f64(v): return struct.pack("<d", v)
def b8(v):  return struct.pack("<b", v)
def pack_pool(strings):
    out = bytearray(i32(len(strings)))
    for s in strings:
        b = s.encode("utf-8"); out += i32(len(b)); out += b
    return bytes(out)

_NON_TOKEN = re.compile(r"[^\w]+", re.UNICODE)
def tokenize(s):
    nfd = unicodedata.normalize("NFD", s.lower())
    stripped = "".join(c for c in nfd if unicodedata.category(c) != "Mn")
    return [p for p in (_NON_TOKEN.sub(" ", stripped)).replace("_", " ").split() if p]

def haversine_m(lat1, lng1, lat2, lng2):
    r = 6371000.0
    d_lat = math.radians(lat2 - lat1); d_lng = math.radians(lng2 - lng1)
    h = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lng / 2) ** 2)
    return r * 2 * math.atan2(math.sqrt(h), math.sqrt(1 - h))

def bb_str(bb):  # Overpass wants (S,W,N,E)
    return f"{bb[1]},{bb[0]},{bb[3]},{bb[2]}"

CACHE_DIR = os.path.join(ROOT, "build", "osm_cache")
def overpass(query):
    os.makedirs(CACHE_DIR, exist_ok=True)
    cf = os.path.join(CACHE_DIR, "ovp_" + hashlib.sha256(query.encode()).hexdigest()[:16] + ".json")
    if os.path.exists(cf):
        print(f"[overpass] cache hit ({os.path.getsize(cf)/1e6:.1f} MB) {os.path.basename(cf)}", flush=True)
        return json.loads(open(cf, "rb").read())
    last = None
    for url in OVERPASS_MIRRORS:
        try:
            print(f"[overpass] {url} …", flush=True)
            req = urllib.request.Request(url, data=query.encode(),
                                         headers={"User-Agent": "navsight-haifa/1.0"})
            raw = urllib.request.urlopen(req, timeout=590).read()
            print(f"[overpass]   received {len(raw)/1e6:.1f} MB", flush=True)
            open(cf, "wb").write(raw)
            return json.loads(raw)
        except Exception as e:  # noqa
            last = e; print(f"[overpass]   failed: {e!r}; next mirror…", flush=True); time.sleep(2)
    raise RuntimeError(f"all Overpass mirrors failed: {last!r}")

# ── ROADS: highways → segments + graph (out geom) ───────────────────────────────
def build_roads(data):
    segments, seg_names, edges = [], [], []
    vlat, vlng, vindex = [], [], {}
    next_seg = 0
    def vertex_of(lat, lng):
        key = (round(lat * 1e7), round(lng * 1e7))
        v = vindex.get(key)
        if v is None:
            v = len(vlat); vlat.append(lat); vlng.append(lng); vindex[key] = v
        return v
    for el in data.get("elements", []):
        if el.get("type") != "way":
            continue
        tags = el.get("tags", {}) or {}
        hw = tags.get("highway"); geom = el.get("geometry")
        if not hw or not geom or HIGHWAY_CODE.get(hw, 0) == 0:
            continue
        if tags.get("access") in ("private", "no"):
            continue
        cc = HIGHWAY_CODE[hw]
        oneway = tags.get("oneway") in ("yes", "true", "1")
        name = tags.get("name")
        prev = None; prev_v = -1
        for p in geom:
            lat = p.get("lat"); lng = p.get("lon")
            if lat is None or lng is None:
                continue
            v = vertex_of(lat, lng)
            if prev is not None:
                segments.append((next_seg, el.get("id", 0), cc, prev[0], prev[1], lat, lng))
                seg_names.append(name)
                edges.append((prev_v, v, haversine_m(prev[0], prev[1], lat, lng), cc, oneway))
                next_seg += 1
            prev = (lat, lng); prev_v = v
    return segments, seg_names, vlat, vlng, edges

# ── SEARCH: named nodes + way centroids → places (out center) ───────────────────
def build_places(data):
    places = []
    def display_name(tags):
        name = tags.get("name") or tags.get("name:en") or tags.get("name:he")
        parts = []
        if name: parts.append(name)
        street = tags.get("addr:street")
        if street:
            house = tags.get("addr:housenumber")
            parts.append((house + " " + street) if house else street)
        city = tags.get("addr:city")
        if city: parts.append(city)
        return ", ".join(parts) if parts else name
    def index_tokens(tags):
        toks, seen = [], set()
        for k in ("name", "name:en", "name:he", "addr:street", "addr:city"):
            if tags.get(k):
                for t in tokenize(tags[k]):
                    if t not in seen:
                        seen.add(t); toks.append(t)
        return toks
    for el in data.get("elements", []):
        tags = el.get("tags", {}) or {}
        disp = display_name(tags)
        if not disp:
            continue
        if el.get("type") == "node":
            lat, lng = el.get("lat"), el.get("lon")
        else:  # way (out center → "center")
            c = el.get("center") or {}
            lat, lng = c.get("lat"), c.get("lon")
        if lat is None or lng is None:
            continue
        places.append((el.get("id", 0), lat, lng, disp, index_tokens(tags)))
    return places

# ── writers (mirror the readers / round-trip test exactly) ──────────────────────
def write_segments(path, segs, seg_names):
    pool, idx = [], {}
    def name_idx(n):
        if n is None: return -1
        i = idx.get(n)
        if i is None: i = len(pool); pool.append(n); idx[n] = i
        return i
    out = bytearray(b"NSSG"); out += i32(SCHEMA_VERSION); out += i32(len(segs))
    for s, nm in zip(segs, seg_names):
        sid, wid, cc, a_lat, a_lng, b_lat, b_lng = s
        out += i64(sid); out += i64(wid); out += i32(cc)
        out += f64(a_lat); out += f64(a_lng); out += f64(b_lat); out += f64(b_lng); out += i32(name_idx(nm))
    out += pack_pool(pool)
    open(path, "wb").write(out); return len(out)

def write_graph(path, vlat, vlng, edges):
    out = bytearray(b"NSGR"); out += i32(SCHEMA_VERSION); out += i32(len(vlat)); out += i32(len(edges))
    for i in range(len(vlat)): out += f64(vlat[i]); out += f64(vlng[i])
    for (a, b, length, cc, ow) in edges:
        out += i32(a); out += i32(b); out += f64(length); out += i32(cc); out += b8(1 if ow else 0)
    open(path, "wb").write(out); return len(out)

def write_geocode(path, places):
    pool, idx = [], {}
    def name_idx(n):
        i = idx.get(n)
        if i is None: i = len(pool); pool.append(n); idx[n] = i
        return i
    postings = {}
    for k, (_pid, _lat, _lng, _disp, tokens) in enumerate(places):
        for t in tokens: postings.setdefault(t, []).append(k)
    out = bytearray(b"NSGC"); out += i32(SCHEMA_VERSION); out += i32(len(places))
    for (pid, lat, lng, disp, _t) in places:
        out += i64(pid); out += f64(lat); out += f64(lng); out += i32(name_idx(disp))
    out += pack_pool(pool)
    out += i32(len(postings))
    for tok, ids in postings.items():
        tb = tok.encode("utf-8"); out += i32(len(tb)); out += tb
        out += i32(len(ids))
        for k in ids: out += i32(k)
    open(path, "wb").write(out); return len(out)

def main():
    # Geocode-only: the SEARCH index (all Israel) is the only embedded blob. ROADS
    # are loaded on demand around the user's location at runtime (RoadRegionManager
    # → Overpass), so they are NOT embedded here. (build_roads/write_segments/
    # write_graph above are retained for the optional embedded-roads path but unused.)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"SEARCH (all Israel {SEARCH_BBOX}) — named features, centroids…")
    names = overpass(f'[out:json][timeout:580];(node["name"]({bb_str(SEARCH_BBOX)});way["name"]({bb_str(SEARCH_BBOX)}););out center;')
    places = build_places(names)
    print(f"      {len(places)} places")
    if not places:
        print("ERROR: no places parsed — aborting.", file=sys.stderr); sys.exit(1)

    geo_b = write_geocode(os.path.join(OUT_DIR, "haifa_geocode.bin"), places)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "bbox": list(SEARCH_BBOX),         # search coverage = default search-anchor centre
        "upstream_pbf_date": time.strftime("%Y-%m-%d"),
        "upstream_pbf_sha": hashlib.sha256(json.dumps(SEARCH_BBOX).encode()).hexdigest()[:16],
        "source": "overpass-api",
        "built_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "place_count": len(places),
        "blob_sizes": {"geocode": geo_b},
        "note": "roads loaded on demand at runtime (RoadRegionManager); not embedded",
    }
    json.dump(manifest, open(os.path.join(OUT_DIR, "haifa_assets.json"), "w", encoding="utf-8"),
              indent=2, ensure_ascii=False)
    print(f"DONE: geocode {geo_b/1e6:.1f} MB ({len(places)} places, all Israel) → {OUT_DIR}")

if __name__ == "__main__":
    main()
