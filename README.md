# 🌊 Sea Router

Generates realistic-looking maritime routes between any two ports on Earth for display on maps. Routes avoid land, traverse canals (Suez, Panama, Kiel), and produce smooth, map-ready GeoJSON polylines.

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

## Architecture

```
┌─────────────────────────────────────────────┐
│  GeoJSON land polygons (OSM / NE)           │
│  ↓                                          │
│  Quadtree subdivision (adaptive depth 16)   │
│  ↓                                          │
│  Coarsening (merge open-ocean cells)        │
│  ↓                                          │
│  Adjacency graph (CSR format)               │
│  + Canal waypoints (Suez, Panama, Kiel)     │
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
- **Canal injection** — manual waypoints for Suez, Panama, and Kiel canals

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
