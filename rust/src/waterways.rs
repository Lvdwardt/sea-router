use serde::Deserialize;

/// A navigable channel the land polygons do not model as water.
///
/// Two kinds end up here. Rivers (Amazon, Elbe, St. Lawrence) are genuinely
/// absent: OSM land polygons are derived from the coastline, so anything above
/// the tidal boundary is land at any resolution. Fjords and straits do exist in
/// the polygons but are narrower than the 0.02° (~2.2km) raster, which seals
/// them shut before the quadtree ever sees them.
///
/// Each corridor is painted into the land raster as water, so the quadtree
/// forms ordinary cells along it and the graph builder wires them up like any
/// other coastal water. Nothing downstream needs to know these are special.
pub struct Waterway {
    pub name: String,
    /// Centerline, [lon, lat] in order. Mouth first, head last, by convention.
    pub points: Vec<[f64; 2]>,
    /// Corridor half-width in km. The painted channel is twice this across.
    pub half_width_km: f64,
}

/// Default corridor half-width when a feature does not set one. 1.2km each side
/// gives a 2.4km channel, one raster cell, the narrowest a corridor can be and
/// still survive rasterization.
const DEFAULT_HALF_WIDTH_KM: f64 = 1.2;

#[derive(Deserialize)]
struct FeatureCollection {
    features: Vec<Feature>,
}

#[derive(Deserialize)]
struct Feature {
    #[serde(default)]
    properties: Props,
    geometry: Geometry,
}

#[derive(Deserialize, Default)]
struct Props {
    #[serde(default)]
    name: Option<String>,
    #[serde(rename = "halfWidthKm", default)]
    half_width_km: Option<f64>,
}

#[derive(Deserialize)]
#[serde(tag = "type")]
enum Geometry {
    LineString { coordinates: Vec<[f64; 2]> },
    MultiLineString { coordinates: Vec<Vec<[f64; 2]>> },
}

/// Load corridors from a GeoJSON FeatureCollection of LineStrings. A missing
/// file is not an error: it just means no corridors.
pub fn load(path: &str) -> Result<Vec<Waterway>, Box<dyn std::error::Error>> {
    if !std::path::Path::new(path).exists() {
        return Ok(Vec::new());
    }
    let fc: FeatureCollection = serde_json::from_str(&std::fs::read_to_string(path)?)?;

    let mut out = Vec::new();
    for f in fc.features {
        let name = f.properties.name.unwrap_or_else(|| "unnamed".into());
        let hw = f.properties.half_width_km.unwrap_or(DEFAULT_HALF_WIDTH_KM);
        let lines = match f.geometry {
            Geometry::LineString { coordinates } => vec![coordinates],
            Geometry::MultiLineString { coordinates } => coordinates,
        };
        for points in lines {
            if points.len() >= 2 {
                out.push(Waterway { name: name.clone(), points, half_width_km: hw });
            }
        }
    }
    Ok(out)
}

/// Content hash, so a cached raster built from different corridors is rejected.
pub fn fingerprint(path: &str) -> u64 {
    let Ok(bytes) = std::fs::read(path) else { return 0 };
    // FNV-1a, enough to detect an edited corridor file.
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::land::LandClassifier;

    fn data_dir() -> String {
        std::env::var("SEA_ROUTER_DATA").unwrap_or_else(|_| "../data".into())
    }

    fn corridors() -> Vec<Waterway> {
        load(&format!("{}/waterways.geojson", data_dir())).expect("waterways.geojson")
    }

    /// Pure geometry, no data files needed beyond the corridor file itself.
    #[test]
    fn corridors_are_well_formed() {
        for w in corridors() {
            assert!(w.points.len() >= 2, "{}: needs at least 2 points", w.name);
            assert!(
                w.half_width_km > 0.0 && w.half_width_km < 25.0,
                "{}: implausible half-width {}km", w.name, w.half_width_km
            );
            for p in &w.points {
                assert!(
                    (-180.0..=180.0).contains(&p[0]) && (-90.0..=90.0).contains(&p[1]),
                    "{}: point out of range {:?}", w.name, p
                );
            }
        }
    }

    /// A corridor that stops short of open water leaves its port connected to
    /// nothing, which is the exact failure the corridors exist to prevent.
    /// Ending on another corridor counts: the Amazon tributaries and Montreal
    /// deliberately hand off rather than each running to the sea.
    #[test]
    #[ignore = "needs the land polygon file; run with --ignored"]
    fn every_corridor_reaches_open_water() {
        let dir = data_dir();
        let classifier = LandClassifier::load(&format!(
            "{}/osm_land_simplified.geojson.json", dir
        ))
        .expect("land data");
        let ways = corridors();

        let mut stranded = Vec::new();
        for w in &ways {
            let end = *w.points.last().unwrap();
            if !classifier.is_land(end[0], end[1]) {
                continue;
            }
            let joins = ways.iter().any(|o| {
                !std::ptr::eq(o, w)
                    && o.points.iter().any(|p| {
                        (p[0] - end[0]).abs() < 1e-4 && (p[1] - end[1]).abs() < 1e-4
                    })
            });
            if !joins {
                stranded.push(format!("  {} ends on land at {:?}", w.name, end));
            }
        }
        assert!(
            stranded.is_empty(),
            "{} corridor(s) end on land and join nothing:\n{}",
            stranded.len(),
            stranded.join("\n"),
        );
    }
}
