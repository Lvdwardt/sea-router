use crate::graph::Graph;
use crate::land::LandClassifier;
use std::collections::BinaryHeap;
use std::time::Instant;

#[allow(dead_code)]
pub struct RouteResult {
    pub path: Vec<[f64; 2]>,
    pub distance_km: f64,
    pub nodes_explored: usize,
    pub time_ms: f64,
}

/// Per-node scratch entry. Keyed by epoch so the whole array can be
/// "cleared" in O(1) by just incrementing `Router::generation`.
struct Entry {
    /// Which generation last wrote this entry. If != Router::generation, treat as fresh.
    epoch: u32,
    g_score: f64,
    came_from: u32,
    closed: bool,
}

/// Reusable A* scratch space. Allocated once at startup, reused every request.
/// Resetting between requests costs O(1) — no memset, no allocation.
pub struct Router {
    entries: Vec<Entry>,
    generation: u32,
}

impl Router {
    pub fn new(node_count: usize) -> Self {
        let entries = (0..node_count)
            .map(|_| Entry { epoch: 0, g_score: f64::INFINITY, came_from: u32::MAX, closed: false })
            .collect();
        Router { entries, generation: 1 }
    }

    /// O(1) reset: bump generation. On the rare u32 wraparound, do a real clear.
    #[inline]
    fn reset(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.generation = 1;
            for e in &mut self.entries {
                e.epoch = 0;
            }
        }
    }

    /// Get a mutable entry, lazily initializing if stale.
    #[inline]
    fn get(&mut self, idx: usize) -> &mut Entry {
        let e = &mut self.entries[idx];
        if e.epoch != self.generation {
            e.epoch = self.generation;
            e.g_score = f64::INFINITY;
            e.came_from = u32::MAX;
            e.closed = false;
        }
        e
    }

    pub fn find_route(
        &mut self,
        graph: &Graph,
        classifier: &LandClassifier,
        from_lon: f64, from_lat: f64,
        to_lon: f64, to_lat: f64,
        coastal_penalty: f32,
    ) -> Option<RouteResult> {
        let t0 = Instant::now();

        let start = graph.find_nearest(from_lon, from_lat);
        let end   = graph.find_nearest(to_lon, to_lat);

        if start == end {
            // Both ports snap to the same graph node. Return the true port
            // coordinates so the drawn line still spans the two ports instead of
            // collapsing onto a single offshore node.
            let mut path = vec![[from_lon, from_lat]];
            if (to_lon - from_lon).abs() > 1e-9 || (to_lat - from_lat).abs() > 1e-9 {
                path.push([to_lon, to_lat]);
            }
            unwrap_lons(&mut path);
            return Some(RouteResult {
                path,
                distance_km: haversine_km(from_lon, from_lat, to_lon, to_lat),
                nodes_explored: 0,
                time_ms: t0.elapsed().as_secs_f64() * 1000.0,
            });
        }

        // O(1) — only touched entries pay initialization cost
        self.reset();

        let end_lon = graph.lon(end);
        let end_lat = graph.lat(end);

        self.get(start).g_score = 0.0;

        let mut open = BinaryHeap::<AStarNode>::new();
        open.push(AStarNode {
            f: haversine_km(graph.lon(start), graph.lat(start), end_lon, end_lat),
            node: start as u32,
        });

        let mut nodes_explored = 0usize;
        const MAX_STEPS: usize = 5_000_000;

        while let Some(AStarNode { node, .. }) = open.pop() {
            let cur = node as usize;

            if cur == end {
                // Reconstruct path
                let mut path = Vec::new();
                let mut id = end;
                loop {
                    path.push([graph.lon(id), graph.lat(id)]);
                    let cf = self.entries[id].came_from;
                    if cf == u32::MAX { break; }
                    id = cf as usize;
                }
                path.reverse();
                // The path begins/ends at the snapped graph nodes, not the real
                // ports. For well-connected coastal ports the snapped node sits
                // metres offshore so nobody noticed; for isolated ports (small
                // islands like Bermuda) it can be hundreds of km away, leaving
                // the line nowhere near the pin. Stitch the true port coordinates
                // onto the ends when the connector stays in open water.
                let connector_km = stitch_endpoints(
                    &mut path, classifier,
                    from_lon, from_lat, to_lon, to_lat,
                );
                unwrap_lons(&mut path);
                return Some(RouteResult {
                    path: douglas_peucker(&path, 0.01),
                    distance_km: self.entries[end].g_score + connector_km,
                    nodes_explored,
                    time_ms: t0.elapsed().as_secs_f64() * 1000.0,
                });
            }

            {
                let e = self.get(cur);
                if e.closed { continue; }
                e.closed = true;
            }

            nodes_explored += 1;
            if nodes_explored >= MAX_STEPS {
                eprintln!("A* cap hit after {nodes_explored} nodes");
                return None;
            }

            let cur_g = self.entries[cur].g_score;

            for (neighbor, base_w, dr) in graph.edges(cur) {
                if self.get(neighbor).closed { continue; }

                let w = edge_weight(base_w, dr, coastal_penalty) as f64;
                let tentative_g = cur_g + w;

                let nb = self.get(neighbor);
                if tentative_g < nb.g_score {
                    nb.g_score = tentative_g;
                    nb.came_from = cur as u32;
                    let h = haversine_km(graph.lon(neighbor), graph.lat(neighbor), end_lon, end_lat);
                    open.push(AStarNode { f: tentative_g + h, node: neighbor as u32 });
                }
            }
        }

        None
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Make a path's longitudes continuous so no consecutive step jumps more than
/// 180°. Antimeridian-crossing paths come out as e.g. 178 → 181 → 183 instead
/// of 178 → -179 → -177, which keeps planar geometry (Douglas-Peucker, LOS,
/// Chaikin) and MapLibre rendering from wrapping the wrong way around the map.
pub fn unwrap_lons(path: &mut [[f64; 2]]) {
    for i in 1..path.len() {
        let mut d = path[i][0] - path[i - 1][0];
        while d > 180.0 { path[i][0] -= 360.0; d -= 360.0; }
        while d < -180.0 { path[i][0] += 360.0; d += 360.0; }
    }
}

/// Distance below which a connector sample is treated as "at the port" and its
/// land hits are ignored. Ports sit on the coast, so the cell containing the
/// port (and the immediate approach) often rasterizes as land — without this
/// tolerance the connector to a coastal port would always be rejected.
const SHORE_TOLERANCE_KM: f64 = 3.0;

/// Prepend the real origin and append the real destination to a routed path
/// when the straight connector from the snapped graph node to the true port
/// stays in open water (ignoring land within `SHORE_TOLERANCE_KM` of the port
/// itself, since ports are coastal). Returns the total km of connector added so
/// the caller can keep `distance_km` honest. A connector that would cross land
/// mid-segment is dropped, leaving the snapped node as the endpoint.
fn stitch_endpoints(
    path: &mut Vec<[f64; 2]>,
    classifier: &LandClassifier,
    from_lon: f64, from_lat: f64,
    to_lon: f64, to_lat: f64,
) -> f64 {
    let mut extra = 0.0;

    if let Some(&first) = path.first() {
        let p = [from_lon, from_lat];
        if !same_point(p, first) && connector_clear(classifier, p, first) {
            extra += haversine_km(p[0], p[1], first[0], first[1]);
            path.insert(0, p);
        }
    }

    if let Some(&last) = path.last() {
        let p = [to_lon, to_lat];
        if !same_point(p, last) && connector_clear(classifier, last, p) {
            extra += haversine_km(last[0], last[1], p[0], p[1]);
            path.push(p);
        }
    }

    extra
}

#[inline]
fn same_point(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9
}

/// True if the straight segment a→b stays in water, sampled ~every 2km. Land
/// hits within `SHORE_TOLERANCE_KM` of either endpoint are ignored so coastal
/// ports remain reachable; interior land hits reject the connector.
fn connector_clear(classifier: &LandClassifier, a: [f64; 2], b: [f64; 2]) -> bool {
    let total = haversine_km(a[0], a[1], b[0], b[1]);
    if total < 1e-6 {
        return true;
    }
    let n = ((total / 2.0).ceil() as usize).max(2);
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let d_from_a = t * total;
        let d_from_b = (1.0 - t) * total;
        if d_from_a <= SHORE_TOLERANCE_KM || d_from_b <= SHORE_TOLERANCE_KM {
            continue;
        }
        let lon = a[0] + t * (b[0] - a[0]);
        let lat = a[1] + t * (b[1] - a[1]);
        if classifier.is_land(lon, lat) {
            return false;
        }
    }
    true
}

/// Haversine distance in km. Longitude delta is wrapped to the short way
/// around the globe so points straddling the antimeridian (e.g. +179°/-179°)
/// measure ~2° apart, not ~358°.
#[inline]
fn haversine_km(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let r = 6371.0_f64;
    let dlat = (lat2 - lat1).to_radians();
    let mut dlon = (lon2 - lon1).abs() % 360.0;
    if dlon > 180.0 { dlon = 360.0 - dlon; }
    let dlon = dlon.to_radians();
    let lat1r = lat1.to_radians();
    let lat2r = lat2.to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1r.cos() * lat2r.cos() * (dlon / 2.0).sin().powi(2);
    r * 2.0 * a.sqrt().asin()
}

/// Quadratic coastal penalty: open-ocean nodes are cheap, tight coastal cells expensive.
#[inline]
fn edge_weight(base_weight: f32, depth_ratio: f32, coastal_penalty: f32) -> f32 {
    let threshold = 0.2_f32;
    if depth_ratio <= threshold {
        base_weight
    } else {
        let t = (depth_ratio - threshold) / (1.0 - threshold);
        base_weight * (1.0 + (coastal_penalty - 1.0) * t * t)
    }
}

#[derive(PartialEq)]
struct AStarNode {
    f: f64,
    node: u32,
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.f.partial_cmp(&self.f).unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Douglas-Peucker simplification (iterative, avoids stack overflow on long routes).
fn douglas_peucker(points: &[[f64; 2]], tolerance: f64) -> Vec<[f64; 2]> {
    let n = points.len();
    if n <= 2 { return points.to_vec(); }

    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;

    let mut stack: Vec<(usize, usize)> = vec![(0, n - 1)];
    while let Some((si, ei)) = stack.pop() {
        if ei <= si + 1 { continue; }
        let (s, e) = (points[si], points[ei]);
        let (mut max_d, mut max_i) = (0.0_f64, si);
        for i in (si + 1)..ei {
            let d = perp_dist(&points[i], &s, &e);
            if d > max_d { max_d = d; max_i = i; }
        }
        if max_d > tolerance {
            keep[max_i] = true;
            stack.push((si, max_i));
            stack.push((max_i, ei));
        }
    }

    points.iter().enumerate().filter(|(i, _)| keep[*i]).map(|(_, p)| *p).collect()
}

#[inline]
fn perp_dist(pt: &[f64; 2], s: &[f64; 2], e: &[f64; 2]) -> f64 {
    let (dx, dy) = (e[0] - s[0], e[1] - s[1]);
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        return ((pt[0] - s[0]).powi(2) + (pt[1] - s[1]).powi(2)).sqrt();
    }
    let t = ((pt[0] - s[0]) * dx + (pt[1] - s[1]) * dy / len_sq).clamp(0.0, 1.0);
    let (px, py) = (s[0] + t * dx, s[1] + t * dy);
    ((pt[0] - px).powi(2) + (pt[1] - py).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// Write a file to the temp dir and return its path.
    fn temp_file(name: &str, contents: &str) -> String {
        let mut path = std::env::temp_dir();
        path.push(name);
        fs::write(&path, contents).unwrap();
        // Drop any stale raster cache so LandClassifier rebuilds from contents.
        let _ = fs::remove_file(format!("{}.raster", path.to_str().unwrap()));
        path.to_str().unwrap().to_string()
    }

    fn all_water() -> LandClassifier {
        let p = temp_file("sr_test_water.geojson.json", r#"{"features":[]}"#);
        LandClassifier::load(&p).unwrap()
    }

    /// Two open-ocean nodes at lat 36, joined by one edge.
    fn two_node_graph() -> Graph {
        let json = r#"{"nodeCount":2,"edgeCount":1,
            "nodes":[-61.0,36.0,1.0,-60.0,36.0,1.0],
            "edges":[0,1,900]}"#;
        let p = temp_file("sr_test_graph.json", json);
        Graph::load(&p).unwrap()
    }

    #[test]
    fn stitches_true_port_coords_over_open_water() {
        let graph = two_node_graph();
        let classifier = all_water();
        let mut router = Router::new(graph.node_count);

        // Ports sit offshore of each snapped node; the gap is open water.
        let from = [-61.2, 36.0];
        let to = [-59.9, 36.0];
        let r = router
            .find_route(&graph, &classifier, from[0], from[1], to[0], to[1], 8.0)
            .expect("route");

        let first = *r.path.first().unwrap();
        let last = *r.path.last().unwrap();
        assert!((first[0] - from[0]).abs() < 1e-6 && (first[1] - from[1]).abs() < 1e-6,
            "path should start at the true origin port, got {:?}", first);
        assert!((last[0] - to[0]).abs() < 1e-6 && (last[1] - to[1]).abs() < 1e-6,
            "path should end at the true destination port, got {:?}", last);
    }

    #[test]
    fn drops_connector_that_would_cross_land() {
        let graph = two_node_graph();
        // Land strip between the origin port (-61.2) and its snapped node (-61.0),
        // clear of both endpoints' 3km shore tolerance.
        let land = r#"{"features":[{"geometry":{"type":"Polygon","coordinates":
            [[[-61.15,35.9],[-61.05,35.9],[-61.05,36.1],[-61.15,36.1],[-61.15,35.9]]]}}]}"#;
        let p = temp_file("sr_test_land_strip.geojson.json", land);
        let classifier = LandClassifier::load(&p).unwrap();
        let mut router = Router::new(graph.node_count);

        let r = router
            .find_route(&graph, &classifier, -61.2, 36.0, -59.9, 36.0, 8.0)
            .expect("route");

        // Front connector crosses land → dropped → path starts at snapped node.
        let first = *r.path.first().unwrap();
        assert!((first[0] - (-61.0)).abs() < 1e-6 && (first[1] - 36.0).abs() < 1e-6,
            "land-crossing connector should be dropped, leaving the snapped node; got {:?}", first);
    }
}
