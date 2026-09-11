# 🌊 Sea Router

Generates realistic-looking maritime routes between any two ports on Earth for display on maps. Routes avoid land, traverse canals (Suez, Panama, Corinth), and produce smooth, map-ready GeoJSON polylines.

## Prerequisites

### Land Data

The routing graph is built from land polygon data. You need to download land data before generating the graph:

**Recommended: OSM Land Polygons (high fidelity, ~313MB)**

```bash
# Download from OpenStreetMap
curl -L "https://osmdata.openstreetmap.de/download/simplified-land-polygons-complete-3857.zip" -o /tmp/land.zip

# Or generate the GeoJSON from Natural Earth (smaller, ~17MB, lower resolution):
# Download ne_10m_land from https://www.naturalearthdata.com/downloads/10m-physical-vectors/
```

Place the GeoJSON file as `data/osm_land_simplified.geojson.json` (preferred) or `data/ne_10m_land.geojson.json` (fallback).

The server will automatically use the OSM file if present, falling back to Natural Earth.

## Quick Start

```bash
# Build (requires Rust)
cd rust && cargo build --release

# Generate the routing graph (one-time, ~5 min for depth 16)
./target/release/sea-router-rs generate 16 ../data

# Start the server
./target/release/sea-router-rs serve ../data
```

Open http://localhost:3001/viewer to visualize routes.

## API

### `GET /route?from=lon,lat&to=lon,lat`

Find a route between two points. Optional `penalty` parameter (default: 5.0) controls coastal avoidance.

```bash
curl "http://localhost:3001/route?from=-1.4,50.9&to=2.17,41.38&penalty=8"
```

Returns GeoJSON FeatureCollection with `raw`, `los` (line-of-sight simplified), and `final` (smoothed) paths.

### `POST /route/multi`

Multi-leg route (cruise itinerary).

```bash
curl -X POST http://localhost:3001/route/multi \
  -H "Content-Type: application/json" \
  -d '{"ports": [[-1.4, 50.9], [2.17, 41.38], [12.5, 41.9]], "penalty": 8}'
```

## Waterway corridors

OSM land polygons are derived from the coastline, and the simplified set this
uses also closes estuaries and harbours. Ports above that line have no water
anywhere near them at any graph resolution: Hamburg on the Elbe, Manaus 1,000km
up the Amazon, Sydney inside its own harbour.

`data/waterways.geojson` lists those channels as LineStrings. The classifier
paints each one into the land raster as water before the quadtree is built, so
ordinary cells form along it and the graph builder wires them up like any other
coastal water. Nothing downstream treats them specially.

```json
{"type": "Feature",
 "properties": {"name": "Hamburg — Elbe", "port": "Hamburg", "halfWidthKm": 1.2},
 "geometry": {"type": "LineString", "coordinates": [[10.005, 53.539], ...]}}
```

Regenerate from the port list and Natural Earth river centerlines:

```bash
curl -L -o /tmp/ne.zip https://naciscdn.org/naturalearth/10m/physical/ne_10m_rivers_lake_centerlines.zip
unzip -q /tmp/ne.zip -d /tmp/ne && ogr2ogr -f GeoJSON /tmp/ne_rivers.geojson /tmp/ne/*.shp
python3 scripts/build-waterways.py
```

Fjords and straits are a different case: they *are* in the land polygons, just
narrower than the raster, so there is no river centerline to follow. Search for
them against the exact geometry instead:

```bash
cd rust && cargo run --release -- find-channels
```

That walks outward from every port the router cannot reach until it meets water
the router can route from, and writes `data/waterways-channels.geojson`.
`build-waterways.py` merges the three sources, in precedence order:

| File | Source | Wins over |
|---|---|---|
| `waterways-manual.geojson` | hand-drawn | everything |
| `waterways-channels.geojson` | `find-channels` | generated |
| `waterways.geojson` | generated output | — |

Edit the manual file for anything the tools get wrong; it is plain GeoJSON, so
geojson.io works. Never edit `waterways.geojson`, it is overwritten. Changing
any of them invalidates the cached raster automatically and requires a graph
regeneration.

## Testing

Unit tests run with no data:

```bash
cd rust && cargo test
```

The port audit needs the full graph and land data, so it is behind `--ignored`:

```bash
cd rust && cargo test --release -- --ignored
```

It checks every port in `rust/tests/fixtures/ports.json` (exported from the
cruisello database by `scripts/export-ports.sh`):

| Test | Asserts |
|---|---|
| `every_port_has_a_clear_connector` | the port attaches to the graph over open water, so the drawn line reaches the pin |
| `every_port_snaps_within_budget` | the attach point is within 100m of the port |
| `no_port_sits_beside_stranded_water` | no port has closer water in a component the router never indexes |
| `raster_cannot_see_the_bosphorus` | pins the raster blind spot the exact test exists to cover |
| `no_port_snap_regressed_against_baseline` | no port got worse than `tests/fixtures/snap-baseline.json` |
| `is_land_precise_matches_known_geography` | the exact land test resolves the Bosphorus, Dardanelles and fjords |

For a ranked report instead of a pass/fail:

```bash
cd rust && cargo run --release -- audit
# --write-baseline freezes the current result as the regression baseline
```

## Architecture

```
┌─────────────────────────────────────────────┐
│  GeoJSON land polygons (OSM / NE)           │
│  + waterways.geojson corridors carved in    │
│  ↓                                          │
│  Quadtree subdivision (adaptive depth 16)   │
│  ↓                                          │
│  Coarsening (merge open-ocean cells)        │
│  ↓                                          │
│  Adjacency graph (CSR format)               │
│  + Canal waypoints (Suez, Panama, Corinth)  │
│  + One node per port, linked over open water│
│  ↓                                          │
│  A* pathfinding + coastal penalty           │
│  ↓                                          │
│  Line-of-sight simplification               │
│  ↓                                          │
│  Chaikin smoothing (land-constrained)       │
└─────────────────────────────────────────────┘
```

**Key optimizations:**
- **CSR graph** — cache-friendly edge traversal
- **R-tree spatial index** — O(log n) nearest-node lookup
- **Connected components** — always route on the main ocean graph
- **Lazy raster cache** — `is_land()` becomes O(1) after first query per cell
- **Douglas-Peucker compression** — reduces raw A* paths by 50%+
- **Segmented bbox rejection** — skips open-ocean LOS checks entirely
- **Canal injection** — manual waypoints for Suez, Panama, and Corinth canals
- **Port nodes** — every port is a graph node, so routes start at the berth
- **Exact land test below 2.2km** — the quadtree uses ring geometry, not the
  raster, once cells are smaller than a raster cell; that is what resolves the
  Bosphorus and the Norwegian fjords

## Performance

Depth 16 graph (~5.3M nodes, ~8.1M edges):

| Route | Time |
|-------|------|
| Southampton → Barcelona | ~30ms |
| Marseille → Shanghai (via Suez) | ~500ms |
| Miami → Nassau | ~1ms |

## Docker

```bash
# Build image (generates graph during build)
docker build -t sea-router .

# Run
docker run -p 3001:3001 sea-router
```

> **Note:** The build downloads OSM land polygons (~873MB) and generates the routing graph during the image build. If you have `data/osm_land_simplified.geojson.json` locally, it'll use that instead (faster rebuild).

## License

MIT
