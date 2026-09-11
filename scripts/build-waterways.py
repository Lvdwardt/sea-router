#!/usr/bin/env python3
"""Generate data/waterways.geojson: navigable corridors the land polygons omit.

OSM land polygons are derived from the coastline, and the simplified set the
router uses also closes estuaries and harbours. Ports above that line (Hamburg
on the Elbe, Manaus on the Amazon, Sydney inside its harbour) end up with no
water anywhere near them, so no graph forms and routes never reach the pin.

Each corridor is a LineString from the port out to real water. Long ones follow
a Natural Earth river centerline downstream to its mouth; short ones cut
straight across a sealed harbour. The router paints them into the land raster
as water, so the quadtree builds ordinary cells along them.

Usage:
    python3 scripts/build-waterways.py [--land PATH] [--rivers PATH] [--out PATH]

Rivers come from Natural Earth 10m rivers + lake centerlines (public domain):
    https://naciscdn.org/naturalearth/10m/physical/ne_10m_rivers_lake_centerlines.zip
    ogr2ogr -f GeoJSON ne_rivers.geojson ne_10m_rivers_lake_centerlines.shp
"""
import argparse, json, math, os, struct, sys
from collections import defaultdict

RASTER_CELL = 0.02
RASTER_COLS = int(360.0 / RASTER_CELL)
RASTER_ROWS = int(180.0 / RASTER_CELL)


class Land:
    """Land lookup backed by the router's own rasterised land bitmap.

    Reading the cache the Rust side already builds keeps this script honest (it
    sees exactly what the quadtree sees) and turns each test into a bit lookup.
    Exact point-in-polygon over the 180k-vertex Eurasia ring would be correct
    too, and far too slow to run for every port.

    The raster must be built WITHOUT waterways.geojson present, or corridors
    from a previous run would make already-carved ports look reachable.
    """

    MAGIC = 0x5345415F52415354
    VERSION = 4

    def __init__(self, raster_path):
        with open(raster_path, "rb") as f:
            blob = f.read()
        magic, version, wcount, fp = struct.unpack_from("<QIIQ", blob, 0)
        if magic != self.MAGIC:
            raise SystemExit(f"{raster_path}: not a sea-router raster")
        if version != self.VERSION:
            raise SystemExit(f"{raster_path}: raster version {version}, expected {self.VERSION}")
        if fp != 0:
            raise SystemExit(
                f"{raster_path}: built with waterways.geojson present (fingerprint {fp:#x}). "
                "Rebuild it in a directory with no corridor file."
            )
        self.bits = blob[24:24 + wcount * 8]

    def is_land(self, lon, lat):
        while lon >= 180.0:
            lon -= 360.0
        while lon < -180.0:
            lon += 360.0
        col = int((lon + 180.0) / RASTER_CELL)
        row = int((lat + 90.0) / RASTER_CELL)
        if not (0 <= col < RASTER_COLS and 0 <= row < RASTER_ROWS):
            return False
        bit = row * RASTER_COLS + col
        return (self.bits[bit // 8] >> (bit % 8)) & 1 == 1


def km(a, b):
    dlat = math.radians(b[1] - a[1])
    dlon = math.radians(b[0] - a[0]) * math.cos(math.radians((a[1] + b[1]) / 2))
    return 6371.0 * math.hypot(dlat, dlon)


def nearest_water(land, lon, lat, max_km):
    """Closest point the polygons call water, or None inside max_km."""
    r = 0.5
    while r <= max_km:
        steps = max(24, min(360, int(r * 8)))
        best = None
        for i in range(steps):
            th = 2 * math.pi * i / steps
            plon = lon + (r * math.cos(th)) / (111.0 * max(0.02, abs(math.cos(math.radians(lat)))))
            plat = lat + (r * math.sin(th)) / 111.0
            if not land.is_land(plon, plat):
                d = km((lon, lat), (plon, plat))
                if best is None or d < best[0]:
                    best = (d, [round(plon, 5), round(plat, 5)])
        if best:
            return best[1]
        r *= 1.4
    return None


def load_rivers(path):
    out = []
    for f in json.load(open(path))["features"]:
        g = f["geometry"]
        lines = [g["coordinates"]] if g["type"] == "LineString" else g["coordinates"]
        for ln in lines:
            if len(ln) >= 2:
                out.append((f["properties"].get("name") or "river", [[c[0], c[1]] for c in ln]))
    return out


class RiverGraph:
    """River vertices as a graph, with fragments stitched back together.

    Natural Earth splits each river into many linestrings and does not digitise
    them in a consistent direction, so neither "follow this line to its end" nor
    "the last vertex is the mouth" works. Searching a joined graph for the
    nearest vertex that is actually in the sea sidesteps both problems.
    """

    JOIN_KM = 3.0
    CELL = 0.05  # degrees per lookup bucket

    def __init__(self, rivers):
        self.pts = []
        self.names = []
        self.adj = defaultdict(set)
        index = {}
        buckets = defaultdict(list)

        def vid(c):
            key = (round(c[0], 4), round(c[1], 4))
            if key not in index:
                index[key] = len(self.pts)
                self.pts.append([key[0], key[1]])
                self.names.append(None)
                buckets[(int(key[0] / self.CELL), int(key[1] / self.CELL))].append(index[key])
            return index[key]

        for name, line in rivers:
            prev = None
            for c in line:
                v = vid(c)
                if self.names[v] is None:
                    self.names[v] = name
                if prev is not None and prev != v:
                    self.adj[prev].add(v)
                    self.adj[v].add(prev)
                prev = v

        # Stitch fragment ends that sit close together but share no vertex.
        self.buckets = buckets
        for v, p in enumerate(self.pts):
            if len(self.adj[v]) > 1:
                continue  # interior vertex, already connected
            for u in self.near(p[0], p[1], self.JOIN_KM):
                if u != v and u not in self.adj[v] and km(p, self.pts[u]) <= self.JOIN_KM:
                    self.adj[v].add(u)
                    self.adj[u].add(v)

    def near(self, lon, lat, radius_km):
        span = int(radius_km / 111.0 / self.CELL) + 1
        gx, gy = int(lon / self.CELL), int(lat / self.CELL)
        out = []
        for i in range(gx - span, gx + span + 1):
            for j in range(gy - span, gy + span + 1):
                out.extend(self.buckets.get((i, j), ()))
        return out

    def nearest(self, lon, lat, max_km):
        best = None
        r = 5.0
        while r <= max_km:
            for v in self.near(lon, lat, r):
                d = km((lon, lat), self.pts[v])
                if d <= max_km and (best is None or d < best[0]):
                    best = (d, v)
            if best:
                return best[1]
            r *= 2
        return None

    def path_to_sea(self, start, land, max_km=4000.0):
        """Shortest walk along the rivers from `start` to open water."""
        import heapq

        dist = {start: 0.0}
        prev = {}
        pq = [(0.0, start)]
        seen_water = set()
        while pq:
            d, v = heapq.heappop(pq)
            if d > dist.get(v, math.inf):
                continue
            if d > max_km:
                break
            if v not in seen_water:
                seen_water.add(v)
                if v != start and not land.is_land(*self.pts[v]):
                    out = [v]
                    while out[-1] in prev:
                        out.append(prev[out[-1]])
                    return list(reversed(out))
            for u in self.adj[v]:
                nd = d + km(self.pts[v], self.pts[u])
                if nd < dist.get(u, math.inf):
                    dist[u] = nd
                    prev[u] = v
                    heapq.heappush(pq, (nd, u))
        return None


def simplify(points, tol_deg=0.004):
    """Douglas-Peucker, to keep corridor files reviewable."""
    if len(points) <= 2:
        return points
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        si, ei = stack.pop()
        if ei <= si + 1:
            continue
        sx, sy = points[si]
        ex, ey = points[ei]
        dx, dy = ex - sx, ey - sy
        ln = math.hypot(dx, dy)
        best_d, best_i = 0.0, si
        for i in range(si + 1, ei):
            px, py = points[i]
            d = abs(dy * px - dx * py + ex * sy - ey * sx) / ln if ln else math.hypot(px - sx, py - sy)
            if d > best_d:
                best_d, best_i = d, i
        if best_d > tol_deg:
            keep[best_i] = True
            stack.append((si, best_i))
            stack.append((best_i, ei))
    return [p for p, k in zip(points, keep) if k]


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--raster", default="/tmp/nocorridor/osm_land_simplified.geojson.json.raster",
                    help="land raster built WITHOUT waterways.geojson present")
    ap.add_argument("--rivers", default="/tmp/ne_rivers.geojson")
    ap.add_argument("--ports", default=os.path.join(root, "rust/tests/fixtures/ports.json"))
    ap.add_argument("--out", default=os.path.join(root, "data/waterways.geojson"))
    ap.add_argument("--manual", default=os.path.join(root, "data/waterways-manual.geojson"),
                    help="hand-drawn corridors; these win over everything")
    ap.add_argument("--channels", default=os.path.join(root, "data/waterways-channels.geojson"),
                    help="output of `sea-router-rs find-channels`; wins over generated")
    ap.add_argument("--scan-km", type=float, default=3.0)
    # A straight line is only safe across a small sealed harbour. Anything
    # longer has to follow a river, or it cuts across whatever land lies between.
    ap.add_argument("--harbour-max-km", type=float, default=12.0)
    ap.add_argument("--attach-km", type=float, default=25.0)
    args = ap.parse_args()

    print("loading land raster…", file=sys.stderr)
    land = Land(args.raster)
    rivers = load_rivers(args.rivers)
    ports = json.load(open(args.ports))
    print("building river graph…", file=sys.stderr)
    rg = RiverGraph(rivers)
    print(
        f"{len(rivers)} river lines, {len(rg.pts)} river vertices, "
        f"{len(ports)} ports",
        file=sys.stderr,
    )

    manual = []
    if os.path.exists(args.manual):
        manual = json.load(open(args.manual))["features"]
        print(f"{len(manual)} hand-drawn corridors kept", file=sys.stderr)
    manual_ports = {f["properties"].get("port") for f in manual}

    # Channels found by searching exact geometry. They cover fjords and straits,
    # which are real water the raster hides, and are trusted over anything this
    # script would guess — but not over a hand-drawn correction.
    channels = []
    if os.path.exists(args.channels):
        channels = [
            f for f in json.load(open(args.channels))["features"]
            if f["properties"].get("port") not in manual_ports
        ]
        print(f"{len(channels)} searched channels kept", file=sys.stderr)
    manual_ports |= {f["properties"].get("port") for f in channels}

    feats, skipped = manual + channels, []
    for p in ports:
        if p["name"] in manual_ports:
            continue
        if nearest_water(land, p["lon"], p["lat"], args.scan_km):
            continue  # water already reachable; no corridor needed

        pts = kind = name = None
        v = rg.nearest(p["lon"], p["lat"], args.attach_km)
        if v is not None:
            walk = rg.path_to_sea(v, land)
            if walk:
                name = rg.names[walk[0]] or "river"
                pts = [[p["lon"], p["lat"]]] + simplify([rg.pts[i] for i in walk])
                kind = "river"
        if pts is None:
            target = nearest_water(land, p["lon"], p["lat"], args.harbour_max_km)
            if not target or km((p["lon"], p["lat"]), target) > args.harbour_max_km:
                skipped.append(p["name"])
                continue
            name, pts, kind = p["name"], [[p["lon"], p["lat"]], target], "harbour"

        feats.append({
            "type": "Feature",
            "properties": {
                "name": f"{p['name']} — {name}" if kind == "river" else f"{p['name']} approach",
                "port": p["name"],
                "kind": kind,
                "halfWidthKm": 1.2,
            },
            "geometry": {"type": "LineString", "coordinates": [[round(c[0], 5), round(c[1], 5)] for c in pts]},
        })
        length = sum(km(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        print(f"  {kind:<8} {p['name']:<30} {len(pts):>4} pts {length:>7.0f} km", file=sys.stderr)

    json.dump(
        {
            "type": "FeatureCollection",
            "_comment": "GENERATED by scripts/build-waterways.py. Hand-drawn "
                        "corridors live in waterways-manual.geojson and are "
                        "copied in here; edit that file, not this one.",
            "features": feats,
        },
        open(args.out, "w"),
        indent=1,
    )
    print(f"\nwrote {len(feats)} corridors -> {args.out}", file=sys.stderr)
    if skipped:
        print(f"no water found for: {', '.join(skipped)}", file=sys.stderr)


if __name__ == "__main__":
    main()
