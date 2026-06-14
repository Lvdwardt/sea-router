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
const RASTER_VERSION: u32 = 3;

pub struct LandClassifier {
    tree: RTree<RingEntry>,
    raster: Vec<u64>,
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
                }
            }
        }

        let tree = RTree::bulk_load(entries);

        // Try loading cached raster first
        let raster_path = format!("{}.raster", path);
        let raster = if let Ok(cached) = Self::load_raster_cache(&raster_path) {
            println!("  Land raster loaded from cache ({} MB).",
                (cached.len() * 8) / 1_048_576);
            cached
        } else {
            println!("  Building land raster {}×{} = {}M cells via scanline (one-time, ~5-15s)...",
                RASTER_COLS, RASTER_ROWS, (RASTER_COLS * RASTER_ROWS) / 1_000_000);

            let raster = Self::build_raster_scanline(&all_rings);

            if let Err(e) = Self::save_raster_cache(&raster_path, &raster) {
                eprintln!("  Warning: could not save raster cache: {}", e);
            } else {
                println!("  Raster cached to: {}", raster_path);
            }
            raster
        };

        Ok(LandClassifier { tree, raster })
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

    fn load_raster_cache(path: &str) -> Result<Vec<u64>, Box<dyn std::error::Error>> {
        let mut f = fs::File::open(path)?;
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)?;

        if buf.len() < 16 { return Err("too small".into()); }
        let magic   = u64::from_le_bytes(buf[0..8].try_into()?);
        let version = u32::from_le_bytes(buf[8..12].try_into()?);
        let wcount  = u32::from_le_bytes(buf[12..16].try_into()?) as usize;

        if magic != RASTER_MAGIC  { return Err("magic mismatch".into()); }
        if version != RASTER_VERSION { return Err("version mismatch".into()); }
        if buf.len() != 16 + wcount * 8 { return Err("size mismatch".into()); }

        let mut raster = vec![0u64; wcount];
        for (i, chunk) in buf[16..].chunks_exact(8).enumerate() {
            raster[i] = u64::from_le_bytes(chunk.try_into()?);
        }
        Ok(raster)
    }

    fn save_raster_cache(path: &str, raster: &[u64]) -> Result<(), Box<dyn std::error::Error>> {
        let mut f = fs::File::create(path)?;
        f.write_all(&RASTER_MAGIC.to_le_bytes())?;
        f.write_all(&RASTER_VERSION.to_le_bytes())?;
        f.write_all(&(raster.len() as u32).to_le_bytes())?;
        for &word in raster { f.write_all(&word.to_le_bytes())?; }
        Ok(())
    }

    pub fn ring_count(&self) -> usize { 0 }

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
}
