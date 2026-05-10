use geo::{Contains, Coord, LineString, Polygon as GeoPolygon};
use rstar::{RTree, RTreeObject, AABB};
use serde::Deserialize;
use std::fs;

/// Land classifier using R-tree indexed polygon rings
/// with a lazily-populated raster cache for fast repeated lookups.
pub struct LandClassifier {
    rings: Vec<GeoPolygon<f64>>,
    tree: RTree<RingEntry>,
    /// Lazily-populated raster cache.
    /// Key: (grid_x, grid_y) packed into u64.
    /// Value: true = land, false = water.
    /// Grid resolution: 0.002° (~222m) — fine enough to detect small islands.
    cache: dashmap::DashMap<u64, bool>,
    grid_w: u64,
    cell_size: f64,
}

struct RingEntry {
    idx: usize,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
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

impl LandClassifier {
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = fs::read_to_string(path)?;
        let geojson: GeoJson = serde_json::from_str(&data)?;

        let mut rings = Vec::new();
        let mut entries = Vec::new();

        for feature in &geojson.features {
            let polygon_groups = match &feature.geometry {
                Geometry::Polygon { coordinates } => vec![coordinates.clone()],
                Geometry::MultiPolygon { coordinates } => coordinates.clone(),
            };

            for poly_coords in polygon_groups {
                if poly_coords.is_empty() { continue; }
                let exterior = &poly_coords[0];
                if exterior.len() < 3 { continue; }

                let ext_coords: Vec<Coord<f64>> = exterior
                    .iter()
                    .map(|c| Coord { x: c[0], y: c[1] })
                    .collect();
                let ext_ls = LineString::new(ext_coords);

                // Remaining rings are holes (interior rings)
                let holes: Vec<LineString<f64>> = poly_coords[1..]
                    .iter()
                    .filter(|r| r.len() >= 3)
                    .map(|r| {
                        LineString::new(
                            r.iter().map(|c| Coord { x: c[0], y: c[1] }).collect(),
                        )
                    })
                    .collect();

                let poly = GeoPolygon::new(ext_ls, holes);

                // Bounding box from exterior ring only
                let mut min_lon = f64::MAX;
                let mut min_lat = f64::MAX;
                let mut max_lon = f64::MIN;
                let mut max_lat = f64::MIN;
                for c in exterior {
                    if c[0] < min_lon { min_lon = c[0]; }
                    if c[1] < min_lat { min_lat = c[1]; }
                    if c[0] > max_lon { max_lon = c[0]; }
                    if c[1] > max_lat { max_lat = c[1]; }
                }

                let idx = rings.len();
                rings.push(poly);
                entries.push(RingEntry { idx, min_lon, min_lat, max_lon, max_lat });
            }
        }

        let tree = RTree::bulk_load(entries);

        let cell_size = 0.002; // ~222m resolution — fine enough for small islands
        let grid_w = (360.0 / cell_size) as u64; // 180000

        Ok(LandClassifier {
            rings,
            tree,
            cache: dashmap::DashMap::with_capacity(2_000_000),
            grid_w,
            cell_size,
        })
    }

    pub fn ring_count(&self) -> usize {
        self.rings.len()
    }

    /// Pack grid coordinates into a single u64 key.
    #[inline]
    fn grid_key(&self, lon: f64, lat: f64) -> u64 {
        let gx = ((lon + 180.0) / self.cell_size) as u64;
        let gy = ((lat + 90.0) / self.cell_size) as u64;
        gy * self.grid_w + gx
    }

    /// Check if a point is on land.
    /// First checks the raster cache (O(1)), falls back to polygon check
    /// and caches the result for future queries.
    #[inline]
    pub fn is_land(&self, lon: f64, lat: f64) -> bool {
        let key = self.grid_key(lon, lat);

        // Cache hit — most common path
        if let Some(val) = self.cache.get(&key) {
            return *val;
        }

        // Cache miss — compute via polygon check
        let result = self.is_land_polygon(lon, lat);
        self.cache.insert(key, result);
        result
    }

    /// Full polygon-based point-in-polygon check.
    /// Uses simple OR: if the point is inside ANY land polygon, it's land.
    /// GeoPolygon::contains handles holes correctly (returns false for points
    /// inside holes), so we don't need XOR winding logic.
    fn is_land_polygon(&self, lon: f64, lat: f64) -> bool {
        let point = geo::Point::new(lon, lat);
        let candidates = self.tree.locate_in_envelope_intersecting(
            &AABB::from_point([lon, lat]),
        );

        for entry in candidates {
            if self.rings[entry.idx].contains(&point) {
                return true; // Inside ANY land polygon = land
            }
        }
        false
    }

    /// Check if a bbox overlaps any land polygon bbox.
    pub fn overlaps_land(&self, min_lon: f64, min_lat: f64, max_lon: f64, max_lat: f64) -> bool {
        let envelope = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);
        self.tree.locate_in_envelope_intersecting(&envelope).next().is_some()
    }
}
