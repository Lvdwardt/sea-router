# ── Stage 1: Build the Rust binary ──
FROM rust:1.85-slim-bookworm AS builder

WORKDIR /build

# Cache dependencies: copy manifests first
COPY rust/Cargo.toml rust/Cargo.lock ./
RUN mkdir src && \
    echo 'fn main() { println!("dummy"); }' > src/main.rs && \
    cargo build --release 2>/dev/null || true && \
    rm -rf src

# Build the real binary
COPY rust/src/ src/
RUN touch src/main.rs && cargo build --release

# ── Stage 2: Prepare land data & generate routing graph ──
FROM debian:bookworm-slim AS generator

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl unzip gdal-bin && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /build/target/release/sea-router-rs /app/sea-router-rs

# Download OSM land polygons (CI always downloads; for local builds with
# pre-existing data, mount or copy into the image after build).
RUN mkdir -p /app/data && \
    echo "Downloading OSM land polygons (~873MB)..." && \
    curl -L --retry 3 --progress-bar \
      "https://osmdata.openstreetmap.de/download/land-polygons-complete-4326.zip" \
      -o /tmp/land.zip && \
    echo "Extracting..." && \
    unzip -q /tmp/land.zip -d /tmp/land && \
    echo "Converting Shapefile to GeoJSON..." && \
    ogr2ogr -f GeoJSON \
      /app/data/osm_land_simplified.geojson.json \
      /tmp/land/land-polygons-complete-4326/land_polygons.shp && \
    rm -rf /tmp/land /tmp/land.zip && \
    echo "Land data ready: $(du -sh /app/data/osm_land_simplified.geojson.json | cut -f1)"

# Generate the routing graph
RUN mkdir -p /app/data/graph && \
    /app/sea-router-rs generate 16 /app/data

# ── Stage 3: Minimal runtime ──
FROM debian:bookworm-slim

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd -r sea && useradd -r -g sea sea

WORKDIR /app

COPY --from=builder /build/target/release/sea-router-rs /app/sea-router-rs
COPY --from=generator /app/data/graph/sea-graph.json /app/data/graph/sea-graph.json
COPY --from=generator /app/data/osm_land_simplified.geojson.json /app/data/osm_land_simplified.geojson.json
COPY viewer.html /app/viewer.html

USER sea

EXPOSE 3001

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -sf http://localhost:3001/route?from=0,51\&to=1,51 || exit 1

CMD ["/app/sea-router-rs", "serve", "/app/data"]
