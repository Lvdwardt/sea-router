use crate::graph::Graph;
use std::collections::BinaryHeap;
use std::time::Instant;

#[allow(dead_code)]
pub struct RouteResult {
    pub path: Vec<[f64; 2]>,
    pub distance_km: f64,
    pub nodes_explored: usize,
    pub time_ms: f64,
}

/// Haversine distance in km.
#[inline]
fn haversine_km(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let r = 6371.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1r = lat1.to_radians();
    let lat2r = lat2.to_radians();
    let a = (dlat / 2.0).sin().powi(2) + lat1r.cos() * lat2r.cos() * (dlon / 2.0).sin().powi(2);
    r * 2.0 * a.sqrt().asin()
}

/// Edge weight with coastal penalty.
/// depth_ratio is high for fine (coastal) cells, low for coarse (open ocean).
/// Quadratic ramp: mid-depth open-sea nodes (between distant islands) stay
/// cheap, while tight coastal cells (depth 14-16) get crushed.
/// Canal waypoints are injected at depth=1, so they're unaffected.
#[inline]
fn edge_weight(base_weight: f32, depth_ratio: f32, coastal_penalty: f32) -> f32 {
    let threshold = 0.2f32;
    if depth_ratio <= threshold {
        // Open ocean — no penalty
        base_weight
    } else {
        // Quadratic ramp — gentle on mid-depth, harsh on extreme coastal
        let t = (depth_ratio - threshold) / (1.0 - threshold);
        let penalty = 1.0 + (coastal_penalty - 1.0) * t * t;
        base_weight * penalty
    }
}

/// A* node for the priority queue.
#[derive(PartialEq)]
struct AStarNode {
    f: f64,
    node: u32,
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse for min-heap
        other.f.partial_cmp(&self.f).unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Find route using A* pathfinding with coastal penalty.
pub fn find_route(
    graph: &Graph,
    from_lon: f64, from_lat: f64,
    to_lon: f64, to_lat: f64,
    coastal_penalty: f32,
) -> Option<RouteResult> {
    let t0 = Instant::now();

    let start = graph.find_nearest(from_lon, from_lat);
    let end = graph.find_nearest(to_lon, to_lat);

    if start == end {
        return Some(RouteResult {
            path: vec![[graph.lon(start), graph.lat(start)]],
            distance_km: 0.0,
            nodes_explored: 0,
            time_ms: t0.elapsed().as_secs_f64() * 1000.0,
        });
    }

    let end_lon = graph.lon(end);
    let end_lat = graph.lat(end);
    let n = graph.node_count;

    let mut g_score = vec![f64::INFINITY; n];
    let mut came_from = vec![u32::MAX; n];
    let mut closed = vec![false; n];

    g_score[start] = 0.0;

    let mut open = BinaryHeap::new();
    open.push(AStarNode {
        f: haversine_km(graph.lon(start), graph.lat(start), end_lon, end_lat),
        node: start as u32,
    });

    let mut nodes_explored = 0usize;

    while let Some(current) = open.pop() {
        let cur = current.node as usize;

        if cur == end {
            // Reconstruct path
            let mut path = Vec::new();
            let mut id = end;
            while id != usize::MAX {
                path.push([graph.lon(id), graph.lat(id)]);
                let cf = came_from[id];
                id = if cf == u32::MAX { usize::MAX } else { cf as usize };
            }
            path.reverse();

            // Compress: remove co-linear waypoints (DP with ~1km tolerance)
            let path = douglas_peucker(&path, 0.01);

            return Some(RouteResult {
                path,
                distance_km: g_score[end],
                nodes_explored,
                time_ms: t0.elapsed().as_secs_f64() * 1000.0,
            });
        }

        if closed[cur] { continue; }
        closed[cur] = true;
        nodes_explored += 1;

        for (neighbor, base_w, dr) in graph.edges(cur) {
            if closed[neighbor] { continue; }

            let w = edge_weight(base_w, dr, coastal_penalty) as f64;
            let tentative_g = g_score[cur] + w;

            if tentative_g < g_score[neighbor] {
                g_score[neighbor] = tentative_g;
                came_from[neighbor] = cur as u32;
                let h = haversine_km(graph.lon(neighbor), graph.lat(neighbor), end_lon, end_lat);
                open.push(AStarNode {
                    f: tentative_g + h,
                    node: neighbor as u32,
                });
            }
        }
    }

    None
}

/// Douglas-Peucker line simplification.
fn douglas_peucker(points: &[[f64; 2]], tolerance: f64) -> Vec<[f64; 2]> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let mut max_dist = 0.0f64;
    let mut max_idx = 0;
    let start = points[0];
    let end = points[points.len() - 1];

    for i in 1..points.len() - 1 {
        let d = perpendicular_dist(&points[i], &start, &end);
        if d > max_dist {
            max_dist = d;
            max_idx = i;
        }
    }

    if max_dist > tolerance {
        let mut left = douglas_peucker(&points[..=max_idx], tolerance);
        let right = douglas_peucker(&points[max_idx..], tolerance);
        left.pop(); // remove duplicate at split point
        left.extend_from_slice(&right);
        left
    } else {
        vec![start, end]
    }
}

#[inline]
fn perpendicular_dist(point: &[f64; 2], start: &[f64; 2], end: &[f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        return ((point[0] - start[0]).powi(2) + (point[1] - start[1]).powi(2)).sqrt();
    }
    let t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let proj_x = start[0] + t * dx;
    let proj_y = start[1] + t * dy;
    ((point[0] - proj_x).powi(2) + (point[1] - proj_y).powi(2)).sqrt()
}
