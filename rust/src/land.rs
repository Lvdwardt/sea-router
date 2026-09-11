use rayon::prelude::*;
use rstar::{RTree, RTreeObject, AABB};
use serde::Deserialize;
use std::fs;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};

// Pre-rasterized land grid.
// 0.02° per cell ≈ 2.2km — fine enough for islands, straits, canals.
// Grid: 18,000 × 9,000 = 162M bits ≈ 20MB.
// Built once via scanline rasterization and cached to disk.
const RASTER_CELL: f64 = 0.02;
const RASTER_COLS: usize = (360.0 / RASTER_CELL) as usize; // 18,000
const RASTER_ROWS: usize = (180.0 / RASTER_CELL) as usize; //  9,000
const RASTER_MAGIC: u64 = 0x5345415F52415354; // "SEA_RAST"
const RASTER_VERSION: u32 = 4;

pub struct LandClassifier {
    tree: RTree<RingEntry>,
    raster: Vec<u64>,
    /// Geometry of *small* (island-scale) rings, for precise point-in-polygon
    /// tests via `is_land_precise`. The 0.02° (~2.2km) raster smears tiny islands
    /// (Bermuda), which breaks short port connectors; precise tests fix that.
    island_rings: Vec<Ring>,
    /// Spatial index over `island_rings` bboxes (value = index into island_rings).
    island_tree: RTree<IslandRing>,
    /// Geometry of the rings too big to bbox-test cheaply — the continents.
    /// Indexing these per ring is useless (one bbox covers Eurasia), so they are
    /// indexed per segment in `coast_segs` and tested by ray casting.
    coast_rings: Vec<Ring>,
    /// Spatial index over every segment of every ring in `coast_rings`.
    coast_segs: RTree<CoastSeg>,
    /// Segments of every carved corridor, for the "is this explicit water"
    /// test. Empty when there is no waterways.geojson.
    corridor_segs: RTree<CorridorSeg>,
}

/// One segment of a carved corridor centerline, with the corridor's half-width.
pub struct CorridorSeg {
    pub a: [f64; 2],
    pub b: [f64; 2],
    pub half_width_deg_lat: f64,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
}

impl RTreeObject for CorridorSeg {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_lon, self.min_lat], [self.max_lon, self.max_lat])
    }
}

/// Max bbox span (degrees) for a ring to count as "island-scale" and be tested
/// by whole-ring containment. Comfortably covers small islands (Bermuda ≈ 0.3°).
/// Bigger rings are the continents and get the per-segment ray cast instead.
const ISLAND_MAX_DEG: f64 = 3.0;

/// One segment of a continental ring, indexed by its own bounding box so a ray
/// cast only touches the segments it could possibly cross.
pub struct CoastSeg {
    pub ring_idx: u32,
    /// Index of the segment's first vertex; the second is `i + 1`.
    pub i: u32,
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

impl RTreeObject for CoastSeg {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_lon, self.min_lat], [self.max_lon, self.max_lat])
    }
}

pub struct IslandRing {
    pub ring_idx: usize,
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

impl RTreeObject for IslandRing {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_lon, self.min_lat], [self.max_lon, self.max_lat])
    }
}

pub struct RingEntry {
    pub idx: usize,
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

impl RTreeObject for RingEntry {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_lon, self.min_lat], [self.max_lon, self.max_lat])
    }
}

#[derive(Deserialize)]
struct GeoJson {
    features: Vec<Feature>,
}

#[derive(Deserialize)]
struct Feature {
    geometry: Geometry,
}

#[derive(Deserialize)]
#[serde(tag = "type")]
enum Geometry {
    Polygon { coordinates: Vec<Vec<[f64; 2]>> },
    MultiPolygon { coordinates: Vec<Vec<Vec<[f64; 2]>>> },
}

/// A ring is just a list of (lon, lat) coordinates (closed polygon ring).
type Ring = Vec<[f64; 2]>;

impl LandClassifier {
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read_to_string(path)?;
        let geojson: GeoJson = serde_json::from_str(&data)?;

        // Collect all rings (exterior + holes) for R-tree and scanline rasterization.
        let mut all_rings: Vec<Ring> = Vec::new();
        let mut entries = Vec::new();
        // Island-scale rings, kept in memory for precise point-in-polygon.
        let mut island_rings: Vec<Ring> = Vec::new();
        let mut island_entries: Vec<IslandRing> = Vec::new();
        // Continental rings, indexed per segment for ray casting.
        let mut coast_rings: Vec<Ring> = Vec::new();
        let mut coast_entries: Vec<CoastSeg> = Vec::new();

        for feature in &geojson.features {
            let polygon_groups = match &feature.geometry {
                Geometry::Polygon { coordinates } => vec![coordinates.clone()],
                Geometry::MultiPolygon { coordinates } => coordinates.clone(),
            };

            for poly_coords in polygon_groups {
                // We rasterize exterior + holes separately using winding parity:
                // Land pixels toggled by each ring (XOR fill = even-odd rule).
                for ring_coords in &poly_coords {
                    if ring_coords.len() < 3 { continue; }

                    let (mut min_lon, mut min_lat) = (f64::MAX, f64::MAX);
                    let (mut max_lon, mut max_lat) = (f64::MIN, f64::MIN);
                    for c in ring_coords {
                        if c[0] < min_lon { min_lon = c[0]; }
                        if c[1] < min_lat { min_lat = c[1]; }
                        if c[0] > max_lon { max_lon = c[0]; }
                        if c[1] > max_lat { max_lat = c[1]; }
                    }

                    let idx = all_rings.len();
                    entries.push(RingEntry { idx, min_lon, min_lat, max_lon, max_lat });
                    all_rings.push(ring_coords.clone());

                    // Keep island-scale rings for precise point-in-polygon tests.
                    if (max_lon - min_lon).max(max_lat - min_lat) <= ISLAND_MAX_DEG {
                        let ring_idx = island_rings.len();
                        island_rings.push(ring_coords.clone());
                        island_entries.push(IslandRing { ring_idx, min_lon, min_lat, max_lon, max_lat });
                    } else {
                        let ring_idx = coast_rings.len() as u32;
                        for i in 0..ring_coords.len() - 1 {
                            let (a, b) = (ring_coords[i], ring_coords[i + 1]);
                            // Segments spanning more than half the globe are
                            // antimeridian wrap artifacts (Antarctica). A ray
                            // cast cannot interpret them; drop them.
                            if (a[0] - b[0]).abs() > 180.0 { continue; }
                            coast_entries.push(CoastSeg {
                                ring_idx,
                                i: i as u32,
                                min_lon: a[0].min(b[0]),
                                min_lat: a[1].min(b[1]),
                                max_lon: a[0].max(b[0]),
                                max_lat: a[1].max(b[1]),
                            });
                        }
                        coast_rings.push(ring_coords.clone());
                    }
                }
            }
        }

        let tree = RTree::bulk_load(entries);
        let island_tree = RTree::bulk_load(island_entries);
        let coast_seg_count = coast_entries.len();
        let coast_segs = RTree::bulk_load(coast_entries);

        // Navigable corridors the polygons do not model. Looked up next to the
        // land file so `generate` and `serve` always agree on them.
        let ways_path = std::path::Path::new(path)
            .parent()
            .map(|d| d.join("waterways.geojson").to_string_lossy().into_owned())
            .unwrap_or_else(|| "waterways.geojson".into());
        let ways = crate::waterways::load(&ways_path).unwrap_or_default();
        let ways_fp = crate::waterways::fingerprint(&ways_path);

        // Try loading cached raster first
        let raster_path = format!("{}.raster", path);
        let raster = if let Ok(cached) = Self::load_raster_cache(&raster_path, ways_fp) {
            println!("  Land raster loaded from cache ({} MB).",
                (cached.len() * 8) / 1_048_576);
            cached
        } else {
            println!("  Building land raster {}×{} = {}M cells via scanline (one-time, ~5-15s)...",
                RASTER_COLS, RASTER_ROWS, (RASTER_COLS * RASTER_ROWS) / 1_000_000);

            let mut raster = Self::build_raster_scanline(&all_rings);

            if !ways.is_empty() {
                let cleared = Self::carve_waterways(&mut raster, &ways);
                println!(
                    "  {} waterway corridors carved ({} raster cells opened)",
                    ways.len(), cleared
                );
            }

            if let Err(e) = Self::save_raster_cache(&raster_path, &raster, ways_fp) {
                eprintln!("  Warning: could not save raster cache: {}", e);
            } else {
                println!("  Raster cached to: {}", raster_path);
            }
            raster
        };

        let mut corridor_entries: Vec<CorridorSeg> = Vec::new();
        for w in &ways {
            // Carving works on whole raster cells, so the channel that actually
            // ends up open is up to half a cell wider than the declared width.
            // Match that here or the two disagree: the router happily follows a
            // carved cell the precise test then calls land.
            let hw = (w.half_width_km + RASTER_CELL * 111.0 / 2.0) / 111.0;
            for seg in w.points.windows(2) {
                let (a, b) = (seg[0], seg[1]);
                corridor_entries.push(CorridorSeg {
                    a, b,
                    half_width_deg_lat: hw,
                    min_lon: a[0].min(b[0]) - hw * 4.0,
                    min_lat: a[1].min(b[1]) - hw,
                    max_lon: a[0].max(b[0]) + hw * 4.0,
                    max_lat: a[1].max(b[1]) + hw,
                });
            }
        }
        let corridor_segs = RTree::bulk_load(corridor_entries);

        println!("  {} island-scale rings kept for precise tests", island_rings.len());
        println!(
            "  {} continental rings indexed as {} segments",
            coast_rings.len(), coast_seg_count
        );
        Ok(LandClassifier {
            tree, raster, island_rings, island_tree, coast_rings, coast_segs, corridor_segs,
        })
    }

    /// Scanline rasterization using even-odd rule.
    ///
    /// For each ring, scan each row in its bbox.
    /// For each row, find all x-crossings of the ring's edges at that row's latitude.
    /// Sort crossings, then fill between pairs (even-odd: toggle land/water).
    /// This is O(total_edges × avg_rows_per_edge) — much faster than per-cell PiP.
    fn build_raster_scanline(rings: &[Ring]) -> Vec<u64> {
        let total_cells = RASTER_COLS * RASTER_ROWS;
        let num_words = (total_cells + 63) / 64;
        let atomic: Vec<AtomicU64> = (0..num_words).map(|_| AtomicU64::new(0)).collect();

        // Process rings in parallel — each ring is independent
        rings.par_iter().for_each(|ring| {
            // Bbox in grid coords
            let (mut min_lat, mut max_lat) = (f64::MAX, f64::MIN);
            for c in ring.iter() {
                if c[1] < min_lat { min_lat = c[1]; }
                if c[1] > max_lat { max_lat = c[1]; }
            }

            let row_start = (((min_lat + 90.0) / RASTER_CELL) as usize).saturating_sub(1);
            let row_end = (((max_lat + 90.0) / RASTER_CELL) as usize + 2).min(RASTER_ROWS);

            for row in row_start..row_end {
                // Latitude of cell center
                let lat = -90.0 + (row as f64 + 0.5) * RASTER_CELL;

                // Find all x-crossings of ring edges at this latitude
                let mut crossings: Vec<f64> = Vec::new();
                let n = ring.len();
                for i in 0..n {
                    let (x0, y0) = (ring[i][0], ring[i][1]);
                    let j = (i + 1) % n;
                    let (x1, y1) = (ring[j][0], ring[j][1]);

                    // Edge crosses this scanline?
                    if (y0 <= lat && y1 > lat) || (y1 <= lat && y0 > lat) {
                        // Linear interpolation of x at lat
                        let t = (lat - y0) / (y1 - y0);
                        crossings.push(x0 + t * (x1 - x0));
                    }
                }

                if crossings.is_empty() { continue; }

                // Sort crossings, fill between pairs (even-odd rule)
                crossings.sort_by(|a, b| a.partial_cmp(b).unwrap());

                for pair in crossings.chunks(2) {
                    if pair.len() < 2 { break; }
                    let col_start = (((pair[0] + 180.0) / RASTER_CELL) as usize).min(RASTER_COLS);
                    let col_end   = (((pair[1] + 180.0) / RASTER_CELL) as usize + 1).min(RASTER_COLS);

                    for col in col_start..col_end {
                        let bit_idx = row * RASTER_COLS + col;
                        // XOR toggle for even-odd fill: holes cancel exterior
                        atomic[bit_idx / 64].fetch_xor(1u64 << (bit_idx % 64), Ordering::Relaxed);
                    }
                }
            }
        });

        atomic.into_iter().map(|a| a.into_inner()).collect()
    }

    fn load_raster_cache(path: &str, want_fp: u64) -> Result<Vec<u64>, Box<dyn std::error::Error>> {
        let mut f = fs::File::open(path)?;
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)?;

        if buf.len() < 24 { return Err("too small".into()); }
        let magic   = u64::from_le_bytes(buf[0..8].try_into()?);
        let version = u32::from_le_bytes(buf[8..12].try_into()?);
        let wcount  = u32::from_le_bytes(buf[12..16].try_into()?) as usize;
        let fp      = u64::from_le_bytes(buf[16..24].try_into()?);

        if magic != RASTER_MAGIC  { return Err("magic mismatch".into()); }
        if version != RASTER_VERSION { return Err("version mismatch".into()); }
        // Editing waterways.geojson must not silently reuse a raster painted
        // from the old corridors.
        if fp != want_fp { return Err("waterway fingerprint mismatch".into()); }
        if buf.len() != 24 + wcount * 8 { return Err("size mismatch".into()); }

        let mut raster = vec![0u64; wcount];
        for (i, chunk) in buf[24..].chunks_exact(8).enumerate() {
            raster[i] = u64::from_le_bytes(chunk.try_into()?);
        }
        Ok(raster)
    }

    fn save_raster_cache(path: &str, raster: &[u64], fp: u64) -> Result<(), Box<dyn std::error::Error>> {
        let mut f = fs::File::create(path)?;
        f.write_all(&RASTER_MAGIC.to_le_bytes())?;
        f.write_all(&RASTER_VERSION.to_le_bytes())?;
        f.write_all(&(raster.len() as u32).to_le_bytes())?;
        f.write_all(&fp.to_le_bytes())?;
        for &word in raster { f.write_all(&word.to_le_bytes())?; }
        Ok(())
    }

    /// Paint each corridor into the raster as water, clearing the land bits in
    /// a band around its centerline. Runs after rasterization so it always
    /// wins: a corridor is an explicit statement that ships pass here.
    fn carve_waterways(raster: &mut [u64], ways: &[crate::waterways::Waterway]) -> usize {
        let mut cleared = 0usize;
        for w in ways {
            for seg in w.points.windows(2) {
                let (a, b) = (seg[0], seg[1]);
                // Step along the segment in units well under one raster cell so
                // the painted band has no gaps.
                let span_deg = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
                let steps = ((span_deg / (RASTER_CELL * 0.4)).ceil() as usize).max(1);
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let lon = a[0] + t * (b[0] - a[0]);
                    let lat = a[1] + t * (b[1] - a[1]);
                    // Convert the half-width to degrees; longitude degrees
                    // shrink with latitude, so widen the column span to match.
                    let half_lat = w.half_width_km / 111.0;
                    let half_lon = half_lat / lat.to_radians().cos().abs().max(0.05);
                    let r0 = (((lat - half_lat + 90.0) / RASTER_CELL) as isize).max(0) as usize;
                    let r1 = (((lat + half_lat + 90.0) / RASTER_CELL) as usize + 1).min(RASTER_ROWS);
                    let c0 = (((lon - half_lon + 180.0) / RASTER_CELL) as isize).max(0) as usize;
                    let c1 = (((lon + half_lon + 180.0) / RASTER_CELL) as usize + 1).min(RASTER_COLS);
                    for row in r0..r1 {
                        for col in c0..c1 {
                            let bit = row * RASTER_COLS + col;
                            let word = &mut raster[bit / 64];
                            let mask = 1u64 << (bit % 64);
                            if *word & mask != 0 {
                                *word &= !mask;
                                cleared += 1;
                            }
                        }
                    }
                }
            }
        }
        cleared
    }

    pub fn ring_count(&self) -> usize { 0 }

    /// True if any carved corridor passes through this cell. The quadtree uses
    /// this to force subdivision: a corridor is ~2.4km wide, far below the
    /// ~19km grid spacing `classify_cell` samples a shallow cell at, so without
    /// it a depth-10 cell over the Amazon reads as solid land, becomes a leaf,
    /// and the corridor inside it is never resolved.
    pub fn overlaps_corridor(&self, min_lon: f64, min_lat: f64, max_lon: f64, max_lat: f64) -> bool {
        let env = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);
        self.corridor_segs.locate_in_envelope_intersecting(&env).next().is_some()
    }

    /// True if the point lies inside a carved corridor, i.e. water we declared
    /// navigable even though the land polygons disagree.
    pub fn in_corridor(&self, lon: f64, lat: f64) -> bool {
        let env = AABB::from_corners([lon, lat], [lon, lat]);
        for seg in self.corridor_segs.locate_in_envelope_intersecting(&env) {
            // Distance in degrees, with longitude scaled by cos(lat) so the
            // corridor is the stated width on the ground rather than on the
            // lon/lat plane.
            let k = lat.to_radians().cos().abs().max(0.05);
            let (px, py) = ((lon - seg.a[0]) * k, lat - seg.a[1]);
            let (dx, dy) = ((seg.b[0] - seg.a[0]) * k, seg.b[1] - seg.a[1]);
            let len2 = dx * dx + dy * dy;
            let t = if len2 == 0.0 { 0.0 } else { ((px * dx + py * dy) / len2).clamp(0.0, 1.0) };
            let (qx, qy) = (px - t * dx, py - t * dy);
            if (qx * qx + qy * qy).sqrt() <= seg.half_width_deg_lat {
                return true;
            }
        }
        false
    }

    /// O(1) bitmap lookup — ~1ns per call.
    /// Longitude is normalized into [-180, 180) so callers may pass
    /// "unwrapped" antimeridian-crossing coordinates (e.g. 181° → -179°).
    #[inline]
    pub fn is_land(&self, lon: f64, lat: f64) -> bool {
        let mut lon = lon;
        while lon >= 180.0 { lon -= 360.0; }
        while lon < -180.0 { lon += 360.0; }
        let col = ((lon + 180.0) / RASTER_CELL) as usize;
        let row = ((lat + 90.0)  / RASTER_CELL) as usize;
        if col >= RASTER_COLS || row >= RASTER_ROWS { return false; }
        let bit_idx = row * RASTER_COLS + col;
        (self.raster[bit_idx / 64] >> (bit_idx % 64)) & 1 == 1
    }

    /// R-tree bbox overlap check — used for the open-ocean fast-path in LOS.
    #[inline]
    pub fn overlaps_land(&self, min_lon: f64, min_lat: f64, max_lon: f64, max_lat: f64) -> bool {
        let envelope = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);
        self.tree.locate_in_envelope_intersecting(&envelope).next().is_some()
    }

    /// Land test that is precise near small islands and raster-backed elsewhere.
    ///
    /// Tiny islands (Bermuda) are smeared by the 0.02° (~2.2km) raster, which
    /// breaks short port connectors. Here we run an exact even-odd
    /// point-in-polygon against the kept island-scale rings; if the point sits
    /// inside one it's land. If no island ring covers the point we fall back to
    /// the raster `is_land` — accurate enough away from tiny features (e.g. on
    /// continental coasts) and cheap. Holes cancel exterior rings via the shared
    /// even-odd toggle, matching the raster fill.
    pub fn is_land_precise(&self, lon: f64, lat: f64) -> bool {
        // A carved corridor is water by decree. Exact ring geometry would say
        // land here (rivers are land in the source polygons), so this has to be
        // checked before anything else or port connectors up a river fail.
        if self.in_corridor(lon, lat) {
            return false;
        }
        let envelope = AABB::from_corners([lon, lat], [lon, lat]);
        let mut inside = false;
        let mut tested_any = false;
        for e in self.island_tree.locate_in_envelope_intersecting(&envelope) {
            tested_any = true;
            let ring = &self.island_rings[e.ring_idx];
            let n = ring.len();
            if n < 3 { continue; }
            let mut j = n - 1;
            for i in 0..n {
                let (xi, yi) = (ring[i][0], ring[i][1]);
                let (xj, yj) = (ring[j][0], ring[j][1]);
                if (yi > lat) != (yj > lat) {
                    let x_cross = (xj - xi) * (lat - yi) / (yj - yi) + xi;
                    if lon < x_cross {
                        inside = !inside;
                    }
                }
                j = i;
            }
        }
        // Continents: ray cast east to the antimeridian, toggling the same
        // parity. Sharing one toggle is what makes a lake (a small hole ring)
        // inside a continent come out as water: two crossings, even, outside.
        let ray = AABB::from_corners([lon, lat], [180.0, lat]);
        for e in self.coast_segs.locate_in_envelope_intersecting(&ray) {
            let ring = &self.coast_rings[e.ring_idx as usize];
            let (xi, yi) = (ring[e.i as usize][0], ring[e.i as usize][1]);
            let (xj, yj) = (ring[e.i as usize + 1][0], ring[e.i as usize + 1][1]);
            if (yi > lat) != (yj > lat) {
                let x_cross = (xj - xi) * (lat - yi) / (yj - yi) + xi;
                if lon < x_cross {
                    inside = !inside;
                    tested_any = true;
                }
            }
        }

        if tested_any {
            // Exact geometry covers this point, and this is exactly where the
            // 2.2km raster is wrong: narrow channels it seals shut, coastlines
            // it smears seaward. `inside` is land, outside is water.
            inside
        } else {
            // Nothing crossed the ray — open ocean east of every landmass.
            // The raster is as good as anything here and far cheaper.
            self.is_land(lon, lat)
        }
    }

    /// True if a *small* (island-scale) land ring intersects the cell — i.e. a
    /// ring whose own bounding box is at most `max_feature_deg` across. Used by
    /// the quadtree to force subdivision around small islands (e.g. Bermuda)
    /// that the coarse grid-sampling in `classify_cell` would otherwise step
    /// over and swallow into open ocean. Big continental rings are ignored here
    /// (their large bbox exceeds the threshold), so this never over-subdivides
    /// open water that merely clips a continent's bbox corner.
    pub fn has_island_feature(
        &self,
        min_lon: f64, min_lat: f64,
        max_lon: f64, max_lat: f64,
        max_feature_deg: f64,
    ) -> bool {
        let envelope = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);
        self.tree
            .locate_in_envelope_intersecting(&envelope)
            .any(|e| (e.max_lon - e.min_lon).max(e.max_lat - e.min_lat) <= max_feature_deg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn temp_file(name: &str, contents: &str) -> String {
        let mut path = std::env::temp_dir();
        path.push(name);
        fs::write(&path, contents).unwrap();
        let _ = fs::remove_file(format!("{}.raster", path.to_str().unwrap()));
        path.to_str().unwrap().to_string()
    }

    #[test]
    fn island_feature_detected_only_for_small_rings() {
        // A small island (~0.1°) near (10,10) and a large landmass (~6°) near (50,0).
        let geo = r#"{"features":[
            {"geometry":{"type":"Polygon","coordinates":[[[10.0,10.0],[10.1,10.0],[10.1,10.1],[10.0,10.1],[10.0,10.0]]]}},
            {"geometry":{"type":"Polygon","coordinates":[[[50.0,0.0],[56.0,0.0],[56.0,6.0],[50.0,6.0],[50.0,0.0]]]}}
        ]}"#;
        let p = temp_file("land_test_islands.geojson.json", geo);
        let c = LandClassifier::load(&p).unwrap();

        // Cell over the small island → island feature present.
        assert!(c.has_island_feature(9.5, 9.5, 10.5, 10.5, 1.0),
            "small island ring must be detected");
        // Cell over the large landmass → ring too big to count as an island.
        assert!(!c.has_island_feature(49.0, -1.0, 57.0, 7.0, 1.0),
            "large continental ring must NOT trip the island detector");
        // Open ocean far from any land → nothing.
        assert!(!c.has_island_feature(-30.0, -30.0, -29.0, -29.0, 1.0),
            "open ocean must have no island feature");
    }


    fn classifier() -> LandClassifier {
        let dir = std::env::var("SEA_ROUTER_DATA").unwrap_or_else(|_| "../data".into());
        LandClassifier::load(&format!("{}/osm_land_simplified.geojson.json", dir))
            .expect("land data not found")
    }

    /// Guards the continental ray cast. Before it existed every one of these
    /// fell back to the 0.02° (~2.2km) raster, which seals the Bosphorus and
    /// the fjords shut and pushes coastlines seaward.
    #[test]
    #[ignore = "needs the land polygon file; run with --ignored"]
    fn is_land_precise_matches_known_geography() {
        let c = classifier();
        // (name, lon, lat, is_land)
        let cases: &[(&str, f64, f64, bool)] = &[
            ("Bosphorus mid-channel",    29.0554,  41.1000, false),
            ("Sea of Marmara",           28.5000,  40.7000, false),
            ("Dardanelles",              26.4870,  40.2300, false),
            ("Geirangerfjord",            6.9000,  62.1000, false),
            ("Puget Sound",            -122.4000,  47.6200, false),
            ("Open Atlantic",           -40.0000,  35.0000, false),
            ("Mid Pacific",            -140.0000,  10.0000, false),
            ("Sahara",                    2.0000,  25.0000, true),
            ("Siberia",                  90.0000,  62.0000, true),
            ("Kansas",                  -98.0000,  38.5000, true),
            ("Amazon rainforest",       -63.0000,  -5.0000, true),
            ("Anatolia",                 33.0000,  39.0000, true),
        ];
        let wrong: Vec<&str> = cases
            .iter()
            .filter(|(_, lon, lat, want)| c.is_land_precise(*lon, *lat) != *want)
            .map(|(n, _, _, _)| *n)
            .collect();
        assert!(wrong.is_empty(), "is_land_precise wrong at: {:?}", wrong);
    }

    /// The raster is what `classify_cell` and the smoothing hot paths use, so
    /// its blind spot is worth pinning: it cannot see water narrower than a
    /// couple of km. This documents the gap the graph still has.
    #[test]
    #[ignore = "needs the land polygon file; run with --ignored"]
    fn raster_cannot_see_the_bosphorus() {
        let c = classifier();
        assert!(
            c.is_land(29.0554, 41.1000),
            "raster unexpectedly resolves the Bosphorus — if the land data or \
             RASTER_CELL changed, the quadtree may now form nodes there and \
             this test should be replaced"
        );
        assert!(!c.is_land_precise(29.0554, 41.1000), "exact geometry has the channel");
    }
}
