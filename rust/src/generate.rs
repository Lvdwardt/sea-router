use crate::land::LandClassifier;
use rstar::RTree;
use std::time::Instant;

/// Cell types in the quadtree.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CellType {
    Water,
    Land,
    Mixed,
}

/// A node in the quadtree. Uses flat arrays for children to avoid boxing overhead.
pub struct QuadCell {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
    pub depth: u8,
    pub cell_type: CellType,
    pub children: Vec<QuadCell>, // empty = leaf, 4 = branch
}

impl QuadCell {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }
}

/// Build the quadtree for the entire world.
pub fn build_quadtree(classifier: &LandClassifier, max_depth: u8) -> QuadCell {
    let t0 = Instant::now();
    println!("  Building quadtree (max depth {})...", max_depth);

    let mut stats = BuildStats { cells: 0, leaves: 0, last_depth: -1 };
    let root = subdivide(classifier, -180.0, -90.0, 180.0, 90.0, 0, max_depth, &mut stats);

    println!(
        "  Quadtree built: {} cells, {} leaves in {:.1}s",
        stats.cells, stats.leaves,
        t0.elapsed().as_secs_f64()
    );
    root
}

struct BuildStats {
    cells: u64,
    leaves: u64,
    last_depth: i8,
}

fn subdivide(
    classifier: &LandClassifier,
    min_lon: f64, min_lat: f64,
    max_lon: f64, max_lat: f64,
    depth: u8, max_depth: u8,
    stats: &mut BuildStats,
) -> QuadCell {
    stats.cells += 1;

    let cell_type = classify_cell(classifier, min_lon, min_lat, max_lon, max_lat);

    // Pure water or pure land → leaf
    if cell_type != CellType::Mixed || depth >= max_depth {
        stats.leaves += 1;

        let final_type = if cell_type == CellType::Mixed {
            let cx = (min_lon + max_lon) / 2.0;
            let cy = (min_lat + max_lat) / 2.0;
            if classifier.is_land(cx, cy) { CellType::Land } else { CellType::Water }
        } else {
            cell_type
        };

        return QuadCell {
            min_lon, min_lat, max_lon, max_lat,
            depth, cell_type: final_type,
            children: Vec::new(),
        };
    }

    // Progress reporting (once per depth level)
    if depth as i8 > stats.last_depth && depth <= 14 {
        stats.last_depth = depth as i8;
        println!("    depth {}: {} cells so far", depth, stats.cells);
    }

    let mid_lon = (min_lon + max_lon) / 2.0;
    let mid_lat = (min_lat + max_lat) / 2.0;

    let children = vec![
        subdivide(classifier, min_lon, min_lat, mid_lon, mid_lat, depth + 1, max_depth, stats), // SW
        subdivide(classifier, mid_lon, min_lat, max_lon, mid_lat, depth + 1, max_depth, stats), // SE
        subdivide(classifier, min_lon, mid_lat, mid_lon, max_lat, depth + 1, max_depth, stats), // NW
        subdivide(classifier, mid_lon, mid_lat, max_lon, max_lat, depth + 1, max_depth, stats), // NE
    ];

    QuadCell {
        min_lon, min_lat, max_lon, max_lat,
        depth, cell_type: CellType::Mixed,
        children,
    }
}

/// Classify a cell using adaptive grid sampling.
/// Spacing capped at 0.25° (~28km) to catch small islands.
fn classify_cell(
    classifier: &LandClassifier,
    min_lon: f64, min_lat: f64,
    max_lon: f64, max_lat: f64,
) -> CellType {
    // Fast path: no land polygons near this cell
    if !classifier.overlaps_land(min_lon, min_lat, max_lon, max_lat) {
        return CellType::Water;
    }

    let cell_w = max_lon - min_lon;
    let cell_h = max_lat - min_lat;
    let max_spacing = 0.25;
    let nx = 3usize.max((cell_w / max_spacing).ceil() as usize + 1);
    let ny = 3usize.max((cell_h / max_spacing).ceil() as usize + 1);

    let mut land_count = 0u32;
    let mut water_count = 0u32;

    for xi in 0..nx {
        let lon = min_lon + (xi as f64 / (nx - 1) as f64) * cell_w;
        for yi in 0..ny {
            let lat = min_lat + (yi as f64 / (ny - 1) as f64) * cell_h;
            if classifier.is_land(lon, lat) {
                land_count += 1;
                if water_count > 0 { return CellType::Mixed; }
            } else {
                water_count += 1;
                if land_count > 0 { return CellType::Mixed; }
            }
        }
    }

    if land_count == 0 { CellType::Water } else { CellType::Land }
}

/// Coarsen open-water cells: merge 4 water siblings when none border land.
pub fn coarsen(root: &mut QuadCell) {
    let t0 = Instant::now();
    let mut total_merges = 0u64;
    let mut pass = 0;

    loop {
        pass += 1;
        let land_bboxes = collect_land_bboxes(root);
        println!("    Building land R-tree ({} entries)...", land_bboxes.len());
        let land_tree = RTree::bulk_load(land_bboxes);
        let merges = coarsen_pass(root, &land_tree);
        total_merges += merges;
        println!("    Pass {}: {} merges", pass, merges);
        if merges == 0 || pass > 20 { break; }
    }

    println!(
        "  Coarsened: {} merges in {} passes ({:.1}s)",
        total_merges, pass, t0.elapsed().as_secs_f64()
    );
}

#[derive(Clone)]
struct LandBbox {
    min_lon: f64, min_lat: f64,
    max_lon: f64, max_lat: f64,
}

impl rstar::RTreeObject for LandBbox {
    type Envelope = rstar::AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_corners([self.min_lon, self.min_lat], [self.max_lon, self.max_lat])
    }
}

fn collect_land_bboxes(root: &QuadCell) -> Vec<LandBbox> {
    let mut result = Vec::new();
    let mut stack = vec![root];
    while let Some(cell) = stack.pop() {
        if cell.is_leaf() {
            if cell.cell_type == CellType::Land {
                result.push(LandBbox {
                    min_lon: cell.min_lon, min_lat: cell.min_lat,
                    max_lon: cell.max_lon, max_lat: cell.max_lat,
                });
            }
        } else {
            for child in &cell.children {
                stack.push(child);
            }
        }
    }
    result
}

fn coarsen_pass(cell: &mut QuadCell, land_tree: &RTree<LandBbox>) -> u64 {
    if cell.is_leaf() { return 0; }

    let mut merges = 0u64;
    for child in cell.children.iter_mut() {
        merges += coarsen_pass(child, land_tree);
    }

    if cell.children.len() != 4 { return merges; }
    let all_water = cell.children.iter().all(|c| c.is_leaf() && c.cell_type == CellType::Water);
    if !all_water { return merges; }

    // Check if any child borders a land cell using R-tree
    let any_touches_land = cell.children.iter().any(|child| {
        let eps = (child.max_lon - child.min_lon) * 0.1;
        let search = rstar::AABB::from_corners(
            [child.min_lon - eps, child.min_lat - eps],
            [child.max_lon + eps, child.max_lat + eps],
        );
        land_tree.locate_in_envelope_intersecting(&search).next().is_some()
    });

    if any_touches_land { return merges; }

    cell.children.clear();
    cell.cell_type = CellType::Water;
    merges + 1
}

/// Collect all water leaf cells.
pub fn collect_water_leaves(root: &QuadCell) -> Vec<WaterLeaf> {
    let mut result = Vec::new();
    let mut stack = vec![root];
    while let Some(cell) = stack.pop() {
        if cell.is_leaf() {
            if cell.cell_type == CellType::Water {
                result.push(WaterLeaf {
                    min_lon: cell.min_lon,
                    min_lat: cell.min_lat,
                    max_lon: cell.max_lon,
                    max_lat: cell.max_lat,
                    depth: cell.depth,
                });
            }
        } else {
            for child in &cell.children {
                stack.push(child);
            }
        }
    }
    result
}

pub struct WaterLeaf {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
    pub depth: u8,
}

/// Count leaves by type.
pub fn count_leaves(root: &QuadCell) -> (u64, u64) {
    let mut water = 0u64;
    let mut land = 0u64;
    let mut stack = vec![root];
    while let Some(cell) = stack.pop() {
        if cell.is_leaf() {
            match cell.cell_type {
                CellType::Water => water += 1,
                CellType::Land => land += 1,
                CellType::Mixed => {}
            }
        } else {
            for child in &cell.children {
                stack.push(child);
            }
        }
    }
    (water, land)
}
