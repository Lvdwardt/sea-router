use axum::{
    Router,
    extract::{Query, State, Json},
    http::StatusCode,
    response::{Html, IntoResponse},
    routing::{get, post},
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Instant;
use tower_http::cors::CorsLayer;

use crate::graph::Graph;
use crate::land::LandClassifier;
use crate::router;
use crate::los;

pub struct AppState {
    pub graph: Graph,
    pub classifier: LandClassifier,
    pub viewer_path: String,
}

#[derive(Deserialize)]
struct RouteQuery {
    from: String,
    to: String,
    penalty: Option<f32>,
}

#[derive(Deserialize)]
struct MultiRouteBody {
    ports: Vec<[f64; 2]>,
    penalty: Option<f32>,
}

#[derive(Serialize)]
struct GeoJsonFeature {
    r#type: &'static str,
    geometry: GeoJsonGeometry,
    properties: serde_json::Value,
}

#[derive(Serialize)]
struct GeoJsonGeometry {
    r#type: &'static str,
    coordinates: Vec<[f64; 2]>,
}

#[derive(Serialize)]
struct GeoJsonCollection {
    r#type: &'static str,
    features: Vec<GeoJsonFeature>,
}

/// Check if a line segment crosses or runs too close to land.
/// Matches graph builder precision: 500m sampling with ±1.5km perpendicular band.
#[inline]
fn segment_crosses_land(from: &[f64; 2], to: &[f64; 2], classifier: &LandClassifier) -> bool {
    let dlon = to[0] - from[0];
    let dlat = to[1] - from[1];
    let approx_km = (dlon * dlon + dlat * dlat).sqrt() * 111.0;
    let n = 3usize.max((approx_km / 0.5).ceil() as usize); // 500m intervals

    let line_len = (dlon * dlon + dlat * dlat).sqrt();
    if line_len < 1e-10 {
        return false;
    }

    // ±1.5km perpendicular band
    let perp_lon = -dlat / line_len * 0.0135;
    let perp_lat = dlon / line_len * 0.0135;

    for i in 1..n {
        let t = i as f64 / n as f64;
        let cx = from[0] + t * dlon;
        let cy = from[1] + t * dlat;
        if classifier.is_land(cx, cy)
            || classifier.is_land(cx + perp_lon, cy + perp_lat)
            || classifier.is_land(cx - perp_lon, cy - perp_lat)
        {
            return true;
        }
    }
    false
}

/// Land-constrained Chaikin corner-cutting with adaptive smoothing.
/// For each segment, tries the standard 25%/75% interpolation first.
/// If that would cross land, tries gentler ratios (10%/90%, then 5%/95%)
/// to produce soft corners near coastlines instead of sharp V-shapes.
fn chaikin_smooth(
    path: &[[f64; 2]],
    iterations: usize,
    classifier: &LandClassifier,
) -> Vec<[f64; 2]> {
    let ratios: &[f64] = &[0.25, 0.10, 0.05];
    let mut result = path.to_vec();
    for _ in 0..iterations {
        if result.len() < 3 {
            break;
        }
        let mut smoothed = Vec::with_capacity(result.len() * 2);
        smoothed.push(result[0]); // keep start
        for i in 0..result.len() - 1 {
            let p0 = result[i];
            let p1 = result[i + 1];

            let mut placed = false;
            for &ratio in ratios {
                let q = [
                    p0[0] * (1.0 - ratio) + p1[0] * ratio,
                    p0[1] * (1.0 - ratio) + p1[1] * ratio,
                ];
                let r = [
                    p0[0] * ratio + p1[0] * (1.0 - ratio),
                    p0[1] * ratio + p1[1] * (1.0 - ratio),
                ];

                let prev = *smoothed.last().unwrap();

                let safe = !classifier.is_land(q[0], q[1])
                    && !classifier.is_land(r[0], r[1])
                    && !segment_crosses_land(&prev, &q, classifier)
                    && !segment_crosses_land(&q, &r, classifier);

                if safe {
                    smoothed.push(q);
                    smoothed.push(r);
                    placed = true;
                    break;
                }
            }

            if !placed {
                // No ratio worked — keep original endpoints
                smoothed.push(p0);
                smoothed.push(p1);
            }
        }
        smoothed.push(result[result.len() - 1]); // keep end

        // Remove consecutive near-duplicates
        smoothed.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9);
        result = smoothed;
    }
    result
}

fn make_feature(coords: Vec<[f64; 2]>, name: &str, color: &str, props: serde_json::Value) -> GeoJsonFeature {
    let mut properties = props;
    properties["name"] = serde_json::json!(name);
    properties["color"] = serde_json::json!(color);
    properties["pointCount"] = serde_json::json!(coords.len());
    GeoJsonFeature {
        r#type: "Feature",
        geometry: GeoJsonGeometry {
            r#type: "LineString",
            coordinates: coords,
        },
        properties,
    }
}

async fn viewer_handler(
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    match std::fs::read_to_string(&state.viewer_path) {
        Ok(html) => Html(html).into_response(),
        Err(_) => (StatusCode::NOT_FOUND, format!("viewer.html not found (tried: {})", state.viewer_path)).into_response(),
    }
}

async fn route_handler(
    State(state): State<Arc<AppState>>,
    Query(params): Query<RouteQuery>,
) -> impl IntoResponse {
    let t0 = Instant::now();

    let from: Vec<f64> = params.from.split(',').filter_map(|s| s.parse().ok()).collect();
    let to: Vec<f64> = params.to.split(',').filter_map(|s| s.parse().ok()).collect();

    if from.len() != 2 || to.len() != 2 {
        return (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": "Invalid coords"}))).into_response();
    }

    let penalty = params.penalty.unwrap_or(5.0);

    let result = router::find_route(&state.graph, from[0], from[1], to[0], to[1], penalty);

    match result {
        None => {
            (StatusCode::OK, Json(serde_json::json!({"error": "No route found"}))).into_response()
        }
        Some(route) => {
            let astar_ms = t0.elapsed().as_secs_f64() * 1000.0;
            let los = los::simplify(&route.path, &state.classifier, 0.5);
            let final_path = chaikin_smooth(&los, 4, &state.classifier);
            let total_ms = t0.elapsed().as_secs_f64() * 1000.0;
            let los_ms = total_ms - astar_ms;

            println!(
                "  Route: raw={} → los={} → final={} pts, {}km (A*={:.0}ms LOS={:.0}ms total={:.0}ms)",
                route.path.len(), los.len(), final_path.len(),
                route.distance_km as u64, astar_ms, los_ms, total_ms
            );

            let collection = GeoJsonCollection {
                r#type: "FeatureCollection",
                features: vec![
                    make_feature(route.path, "raw", "#ff6600", serde_json::json!({
                        "distanceKm": route.distance_km as u64,
                        "nodesExplored": route.nodes_explored,
                        "timeMs": total_ms as u64,
                    })),
                    make_feature(los, "los", "#00ccff", serde_json::json!({})),
                    make_feature(final_path, "final", "#ff4444", serde_json::json!({})),
                ],
            };

            (StatusCode::OK, Json(serde_json::json!(collection))).into_response()
        }
    }
}

async fn multi_route_handler(
    State(state): State<Arc<AppState>>,
    Json(body): Json<MultiRouteBody>,
) -> impl IntoResponse {
    let t0 = Instant::now();

    if body.ports.len() < 2 {
        return (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": "Need ≥2 ports"}))).into_response();
    }

    let penalty = body.penalty.unwrap_or(5.0);
    let mut raw_all: Vec<[f64; 2]> = Vec::new();
    let mut los_all: Vec<[f64; 2]> = Vec::new();
    let mut final_all: Vec<[f64; 2]> = Vec::new();
    let mut total_distance = 0.0f64;
    let mut total_nodes = 0usize;

    for i in 0..body.ports.len() - 1 {
        let from = body.ports[i];
        let to = body.ports[i + 1];

        let result = router::find_route(&state.graph, from[0], from[1], to[0], to[1], penalty);
        match result {
            None => {
                return (StatusCode::OK, Json(serde_json::json!({
                    "error": format!("No route for leg {}", i + 1)
                }))).into_response();
            }
            Some(route) => {
                let los = los::simplify(&route.path, &state.classifier, 0.5);
                let final_path = chaikin_smooth(&los, 4, &state.classifier);

                let skip = if i == 0 { 0 } else { 1 };
                raw_all.extend_from_slice(&route.path[skip..]);
                los_all.extend_from_slice(&los[skip..]);
                final_all.extend_from_slice(&final_path[skip..]);
                total_distance += route.distance_km;
                total_nodes += route.nodes_explored;
            }
        }
    }

    let total_ms = t0.elapsed().as_secs_f64() * 1000.0;

    println!(
        "  Multi: {} legs, raw={} → los={} → final={} pts, {}km in {:.0}ms",
        body.ports.len() - 1, raw_all.len(), los_all.len(), final_all.len(),
        total_distance as u64, total_ms
    );

    let collection = GeoJsonCollection {
        r#type: "FeatureCollection",
        features: vec![
            make_feature(raw_all, "raw", "#ff6600", serde_json::json!({
                "distanceKm": total_distance as u64,
                "nodesExplored": total_nodes,
                "timeMs": total_ms as u64,
                "legs": body.ports.len() - 1,
            })),
            make_feature(los_all, "los", "#00ccff", serde_json::json!({})),
            make_feature(final_all, "final", "#ff4444", serde_json::json!({})),
        ],
    };

    (StatusCode::OK, Json(serde_json::json!(collection))).into_response()
}

pub async fn run(state: Arc<AppState>) {
    let cors = CorsLayer::permissive();

    let app = Router::new()
        .route("/", get(viewer_handler))
        .route("/viewer", get(viewer_handler))
        .route("/route", get(route_handler))
        .route("/route/multi", post(multi_route_handler))
        .layer(cors)
        .with_state(state);

    let port = std::env::var("PORT").unwrap_or_else(|_| "3001".into());
    let addr = format!("0.0.0.0:{}", port);
    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    println!("   Viewer: http://localhost:{}/viewer", port);
    axum::serve(listener, app).await.unwrap();
}
