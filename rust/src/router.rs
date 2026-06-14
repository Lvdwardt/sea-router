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
        from_lon: f64, from_lat: f64,
        to_lon: f64, to_lat: f64,
        coastal_penalty: f32,
    ) -> Option<RouteResult> {
        let t0 = Instant::now();

        let start = graph.find_nearest(from_lon, from_lat);
        let end   = graph.find_nearest(to_lon, to_lat);

        if start == end {
            return Some(RouteResult {
                path: vec![[graph.lon(start), graph.lat(start)]],
                distance_km: 0.0,
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
                unwrap_lons(&mut path);
                return Some(RouteResult {
                    path: douglas_peucker(&path, 0.01),
                    distance_km: self.entries[end].g_score,
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
