use rstar::{RTree, RTreeObject, AABB, PointDistance};
use serde::Deserialize;
use std::fs;

/// Flat graph loaded from sea-graph.json.
/// Nodes: [lon0, lat0, depth0, lon1, lat1, depth1, ...]
/// Edges stored as CSR (Compressed Sparse Row) for cache-friendly access.
#[allow(dead_code)]
pub struct Graph {
    pub node_count: usize,
    pub edge_count: usize,
    pub max_depth: f64,
    /// [lon, lat, depth] × node_count
    pub nodes: Vec<f64>,
    /// CSR: offsets[i]..offsets[i+1] are the edges for node i
    pub offsets: Vec<usize>,
    /// CSR edge targets
    pub targets: Vec<u32>,
    /// CSR edge base weights (km × 10)
    pub base_weights: Vec<f32>,
    /// CSR edge depth ratios (pre-computed)
    pub depth_ratios: Vec<f32>,
    /// Spatial index for nearest-node lookup
    pub spatial: RTree<NodeEntry>,
}

#[derive(Clone)]
pub struct NodeEntry {
    pub id: u32,
    pub lon: f64,
    pub lat: f64,
}

impl RTreeObject for NodeEntry {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.lon, self.lat])
    }
}

impl PointDistance for NodeEntry {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.lon - point[0];
        let dy = self.lat - point[1];
        dx * dx + dy * dy
    }
}

#[derive(Deserialize)]
#[allow(dead_code)]
struct RawGraph {
    #[serde(rename = "nodeCount")]
    node_count: usize,
    #[serde(rename = "edgeCount")]
    edge_count: usize,
    nodes: Vec<f64>,
    edges: Vec<f64>,
}

impl Graph {
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read_to_string(path)?;
        let raw: RawGraph = serde_json::from_str(&data)?;

        let node_count = raw.node_count;
        let nodes = raw.nodes;

        // Find max depth
        let mut max_depth: f64 = 0.0;
        for i in 0..node_count {
            let d = nodes[i * 3 + 2];
            if d > max_depth { max_depth = d; }
        }

        // Build CSR adjacency from flat edge array [from, to, weight×10, ...]
        let edge_count = raw.edges.len() / 3;
        
        // Count degree per node
        let mut degree = vec![0u32; node_count];
        for i in 0..edge_count {
            let from = raw.edges[i * 3] as usize;
            let to = raw.edges[i * 3 + 1] as usize;
            degree[from] += 1;
            degree[to] += 1; // undirected
        }

        // Build offsets
        let mut offsets = vec![0usize; node_count + 1];
        for i in 0..node_count {
            offsets[i + 1] = offsets[i] + degree[i] as usize;
        }
        let total_edges = offsets[node_count];

        let mut targets = vec![0u32; total_edges];
        let mut base_weights = vec![0.0f32; total_edges];
        let mut depth_ratios = vec![0.0f32; total_edges];
        let mut cursor = vec![0u32; node_count]; // current write position per node

        for i in 0..edge_count {
            let from = raw.edges[i * 3] as usize;
            let to = raw.edges[i * 3 + 1] as usize;
            let w = raw.edges[i * 3 + 2] as f32 / 10.0;

            let depth_from = nodes[from * 3 + 2];
            let depth_to = nodes[to * 3 + 2];
            // Use MAX depth — an edge touching any coastal node gets the full penalty.
            // Average was letting edges from open ocean to coast slip through cheaply.
            let dr = (depth_from.max(depth_to) / max_depth) as f32;

            // Forward edge
            let pos = offsets[from] + cursor[from] as usize;
            targets[pos] = to as u32;
            base_weights[pos] = w;
            depth_ratios[pos] = dr;
            cursor[from] += 1;

            // Reverse edge
            let pos = offsets[to] + cursor[to] as usize;
            targets[pos] = from as u32;
            base_weights[pos] = w;
            depth_ratios[pos] = dr;
            cursor[to] += 1;
        }

        // Find connected components via BFS — only index the largest component
        let mut component = vec![u32::MAX; node_count];
        let mut comp_id = 0u32;
        let mut comp_sizes: Vec<usize> = Vec::new();

        for start in 0..node_count {
            if component[start] != u32::MAX { continue; }
            if offsets[start + 1] == offsets[start] { continue; } // isolated

            let mut size = 0usize;
            let mut queue = std::collections::VecDeque::new();
            queue.push_back(start);
            component[start] = comp_id;

            while let Some(node) = queue.pop_front() {
                size += 1;
                for idx in offsets[node]..offsets[node + 1] {
                    let neighbor = targets[idx] as usize;
                    if component[neighbor] == u32::MAX {
                        component[neighbor] = comp_id;
                        queue.push_back(neighbor);
                    }
                }
            }

            comp_sizes.push(size);
            comp_id += 1;
        }

        // Find largest component
        let main_comp = comp_sizes.iter().enumerate()
            .max_by_key(|(_, s)| *s)
            .map(|(i, _)| i as u32)
            .unwrap_or(0);

        let main_size = comp_sizes.get(main_comp as usize).copied().unwrap_or(0);
        println!("  {} components, largest: {} nodes ({:.1}%)",
            comp_sizes.len(), main_size,
            main_size as f64 / node_count as f64 * 100.0);

        // Build spatial index — only nodes in the main component
        let mut entries = Vec::with_capacity(main_size);
        for i in 0..node_count {
            if component[i] == main_comp {
                entries.push(NodeEntry {
                    id: i as u32,
                    lon: nodes[i * 3],
                    lat: nodes[i * 3 + 1],
                });
            }
        }
        let spatial = RTree::bulk_load(entries);

        println!("  maxDepth={}", max_depth);

        Ok(Graph {
            node_count,
            edge_count,
            max_depth,
            nodes,
            offsets,
            targets,
            base_weights,
            depth_ratios,
            spatial,
        })
    }

    #[inline]
    pub fn lon(&self, id: usize) -> f64 { self.nodes[id * 3] }
    #[inline]
    pub fn lat(&self, id: usize) -> f64 { self.nodes[id * 3 + 1] }
    #[inline]
    #[allow(dead_code)]
    pub fn depth(&self, id: usize) -> f64 { self.nodes[id * 3 + 2] }

    /// Find nearest connected node using R-tree (O(log n)).
    pub fn find_nearest(&self, lon: f64, lat: f64) -> usize {
        self.spatial
            .nearest_neighbor(&[lon, lat])
            .map(|e| e.id as usize)
            .unwrap_or(0)
    }

    /// Iterate edges of a node.
    #[inline]
    pub fn edges(&self, node: usize) -> impl Iterator<Item = (usize, f32, f32)> + '_ {
        let start = self.offsets[node];
        let end = self.offsets[node + 1];
        (start..end).map(move |i| {
            (self.targets[i] as usize, self.base_weights[i], self.depth_ratios[i])
        })
    }
}
