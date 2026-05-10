use crate::generate::WaterLeaf;
use crate::land::LandClassifier;
use rstar::{RTree, RTreeObject, AABB};
use std::collections::HashSet;
use std::time::Instant;

/// Graph node from a water leaf cell.
struct GNode {
    lon: f64,
    lat: f64,
    depth: u8,
}

/// R-tree entry for spatial adjacency search.
#[derive(Clone)]
struct CellEntry {
    id: u32,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
}

impl RTreeObject for CellEntry {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.min_lon, self.min_lat],
            [self.max_lon, self.max_lat],
        )
    }
}

/// Haversine distance in km.
fn haversine_km(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let r = 6371.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    r * 2.0 * a.sqrt().asin()
}

/// Check if two cells are adjacent (touching edges or corners).
fn cells_adjacent(a: &WaterLeaf, b: &WaterLeaf) -> bool {
    let eps = (a.max_lon - a.min_lon).min(b.max_lon - b.min_lon) * 0.5;

    let overlap_lon = a.max_lon >= b.min_lon - eps && b.max_lon >= a.min_lon - eps;
    let overlap_lat = a.max_lat >= b.min_lat - eps && b.max_lat >= a.min_lat - eps;

    if !overlap_lon || !overlap_lat {
        return false;
    }

    // Not the same cell
    let dx = ((a.min_lon + a.max_lon) / 2.0 - (b.min_lon + b.max_lon) / 2.0).abs();
    let dy = ((a.min_lat + a.max_lat) / 2.0 - (b.min_lat + b.max_lat) / 2.0).abs();
    dx >= 0.00001 || dy >= 0.00001
}

/// Check if an edge crosses or runs too close to land.
/// Samples every ~500m along the edge with ±1km and ±2km perpendicular band.
/// This catches both direct land crossings AND edges that run parallel to
/// coastlines within 2km. For short edges (<2km, typical of port-area cells),
/// the perpendicular offset is scaled down proportionally to keep ports accessible.
fn edge_crosses_land(
    lon1: f64, lat1: f64,
    lon2: f64, lat2: f64,
    classifier: &LandClassifier,
) -> bool {
    let dlon = lon2 - lon1;
    let dlat = lat2 - lat1;
    let approx_km = (dlon * dlon + dlat * dlat).sqrt() * 111.0;
    let n = 3usize.max((approx_km / 0.5).ceil() as usize); // 500m intervals

    let line_len = (dlon * dlon + dlat * dlat).sqrt();
    if line_len < 1e-10 {
        return false;
    }

    // Perpendicular unit vector
    let perp_lon = -dlat / line_len;
    let perp_lat = dlon / line_len;

    // Scale buffer by edge length: full 1.5km for long edges, proportional for short
    let scale = (approx_km / 2.0).min(1.0); // ramps from 0 at 0km to 1.0 at 2km+
    let offset = 0.0135 * scale;  // ~1.5km

    for i in 1..n {
        let t = i as f64 / n as f64;
        let cx = lon1 + t * dlon;
        let cy = lat1 + t * dlat;

        // Center line
        if classifier.is_land(cx, cy) {
            return true;
        }
        // ±1km band
        if classifier.is_land(cx + perp_lon * offset, cy + perp_lat * offset)
            || classifier.is_land(cx - perp_lon * offset, cy - perp_lat * offset)
        {
            return true;
        }
    }
    false
}

/// Output format matching the TypeScript sea-graph.json.
pub struct GraphOutput {
    pub node_count: usize,
    pub edge_count: usize,
    pub nodes: Vec<f64>,  // [lon, lat, depth, ...]
    pub edges: Vec<f64>,  // [from, to, weight*10, ...]
}

/// Build the adjacency graph from water leaves.
pub fn build_graph(leaves: &[WaterLeaf], classifier: &LandClassifier) -> GraphOutput {
    let t0 = Instant::now();

    // Create nodes from centroids
    let nodes: Vec<GNode> = leaves.iter().map(|l| GNode {
        lon: (l.min_lon + l.max_lon) / 2.0,
        lat: (l.min_lat + l.max_lat) / 2.0,
        depth: l.depth,
    }).collect();

    println!("  {} water nodes", nodes.len());

    // Build R-tree for spatial adjacency lookup
    println!("  Building spatial index for adjacency...");
    let entries: Vec<CellEntry> = leaves.iter().enumerate().map(|(i, l)| CellEntry {
        id: i as u32,
        min_lon: l.min_lon,
        min_lat: l.min_lat,
        max_lon: l.max_lon,
        max_lat: l.max_lat,
    }).collect();
    let tree = RTree::bulk_load(entries);

    // Find edges
    println!("  Finding adjacent cells...");
    let mut edge_set: HashSet<u64> = HashSet::new();
    let mut raw_edges: Vec<(u32, u32, f64)> = Vec::new();
    let mut land_crossings = 0u64;

    for i in 0..leaves.len() {
        let leaf = &leaves[i];
        let eps = ((leaf.max_lon - leaf.min_lon) * 0.01).max(0.0001);

        let search_env = AABB::from_corners(
            [leaf.min_lon - eps, leaf.min_lat - eps],
            [leaf.max_lon + eps, leaf.max_lat + eps],
        );

        for entry in tree.locate_in_envelope_intersecting(&search_env) {
            let j = entry.id as usize;
            if j <= i { continue; }

            if !cells_adjacent(&leaves[i], &leaves[j]) { continue; }

            let key = (i as u64) << 32 | (j as u64);
            if edge_set.contains(&key) { continue; }
            edge_set.insert(key);

            let n1 = &nodes[i];
            let n2 = &nodes[j];

            if edge_crosses_land(n1.lon, n1.lat, n2.lon, n2.lat, classifier) {
                land_crossings += 1;
                continue;
            }

            let dist = haversine_km(n1.lon, n1.lat, n2.lon, n2.lat);
            raw_edges.push((i as u32, j as u32, dist));
        }

        if i > 0 && i % 500_000 == 0 {
            println!("    {}/{} nodes, {} edges", i, leaves.len(), raw_edges.len());
        }
    }

    if land_crossings > 0 {
        println!("  Removed {} edges crossing land", land_crossings);
    }

    println!(
        "  Base graph: {} nodes, {} edges in {:.1}s",
        nodes.len(), raw_edges.len(),
        t0.elapsed().as_secs_f64()
    );

    // Pack into flat arrays
    let mut flat_nodes = Vec::with_capacity(nodes.len() * 3);
    for n in &nodes {
        flat_nodes.push((n.lon * 100000.0).round() / 100000.0);
        flat_nodes.push((n.lat * 100000.0).round() / 100000.0);
        flat_nodes.push(n.depth as f64);
    }

    let mut flat_edges = Vec::with_capacity(raw_edges.len() * 3);
    for (from, to, weight) in &raw_edges {
        flat_edges.push(*from as f64);
        flat_edges.push(*to as f64);
        flat_edges.push((*weight * 10.0).round());
    }

    // ── Inject canal waypoints ──
    println!("  Injecting canal waypoints...");
    let mut canal_nodes_added = 0usize;
    let mut canal_edges_added = 0usize;

    // Build a simple spatial lookup for existing nodes to connect canal ends
    use rstar::PointDistance;

    #[derive(Clone)]
    struct NodePt { id: u32, lon: f64, lat: f64 }
    impl RTreeObject for NodePt {
        type Envelope = AABB<[f64; 2]>;
        fn envelope(&self) -> Self::Envelope { AABB::from_point([self.lon, self.lat]) }
    }
    impl PointDistance for NodePt {
        fn distance_2(&self, point: &[f64; 2]) -> f64 {
            let dx = self.lon - point[0];
            let dy = self.lat - point[1];
            dx * dx + dy * dy
        }
    }

    let node_pts: Vec<NodePt> = nodes.iter().enumerate().map(|(i, n)| NodePt {
        id: i as u32, lon: n.lon, lat: n.lat
    }).collect();
    let node_tree = RTree::bulk_load(node_pts);

    for canal in crate::canals::CANALS {
        let wp = canal.waypoints;
        if wp.len() < 2 { continue; }

        let _base_id = flat_nodes.len() / 3;
        let mut canal_ids: Vec<usize> = Vec::new();

        // Add canal waypoints as new nodes (depth 1 = open ocean, no penalty)
        for pt in wp {
            let id = flat_nodes.len() / 3;
            flat_nodes.push((pt[0] * 100000.0).round() / 100000.0);
            flat_nodes.push((pt[1] * 100000.0).round() / 100000.0);
            flat_nodes.push(1.0); // depth 1 = treated as open ocean
            canal_ids.push(id);
            canal_nodes_added += 1;
        }

        // Chain canal waypoints together
        for i in 0..canal_ids.len() - 1 {
            let a = canal_ids[i];
            let b = canal_ids[i + 1];
            let dist = haversine_km(wp[i][0], wp[i][1], wp[i + 1][0], wp[i + 1][1]);
            flat_edges.push(a as f64);
            flat_edges.push(b as f64);
            flat_edges.push((dist * 10.0).round());
            canal_edges_added += 1;
        }

        // Connect first and last canal waypoints to nearest existing graph nodes
        for &canal_idx in &[0, wp.len() - 1] {
            let pt = wp[canal_idx];
            let canal_id = canal_ids[canal_idx];

            // Find k nearest existing nodes and connect to them
            let nearest: Vec<_> = node_tree.nearest_neighbor_iter(&[pt[0], pt[1]])
                .take(5)
                .collect();

            for nn in nearest {
                let dist = haversine_km(pt[0], pt[1], nn.lon, nn.lat);
                if dist < 100.0 { // only connect if within 100km
                    flat_edges.push(canal_id as f64);
                    flat_edges.push(nn.id as f64);
                    flat_edges.push((dist * 10.0).round());
                    canal_edges_added += 1;
                }
            }
        }

        println!("    {} — {} waypoints injected", canal.name, wp.len());
    }

    let total_nodes = flat_nodes.len() / 3;
    let total_edges = flat_edges.len() / 3;

    println!(
        "  Graph: {} nodes (+{}), {} edges (+{}) in {:.1}s",
        total_nodes, canal_nodes_added,
        total_edges, canal_edges_added,
        t0.elapsed().as_secs_f64()
    );

    GraphOutput {
        node_count: total_nodes,
        edge_count: total_edges,
        nodes: flat_nodes,
        edges: flat_edges,
    }
}
