# ── Stage 1: Build the Rust binary ──────────────────────────────────────────
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

# ── Stage 2: Minimal runtime ─────────────────────────────────────────────────
FROM debian:bookworm-slim

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd -r sea && useradd -r -g sea sea

WORKDIR /app

COPY --from=builder /build/target/release/sea-router-rs /app/sea-router-rs

# Pre-built graph and land data — generated once via generate-graph workflow,
# provided in the build context by the CI workflow.
COPY data/graph/sea-graph.json /app/data/graph/sea-graph.json

COPY viewer.html /app/viewer.html

USER sea

EXPOSE 3001

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -sf http://localhost:3001/route?from=0,51\&to=1,51 || exit 1

CMD ["/app/sea-router-rs", "serve", "/app/data"]
