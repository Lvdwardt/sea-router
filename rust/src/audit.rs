use serde::{Deserialize, Serialize};

use crate::graph::Graph;
use crate::land::LandClassifier;
use crate::router::{haversine_km, snap_to_water};
use rstar::RTree;
use crate::graph::NodeEntry;

/// One port from the itinerary database, as exported by
/// `scripts/export-ports.sh` into `rust/tests/fixtures/ports.json`.
#[derive(Deserialize, Clone)]
pub struct Port {
    pub name: String,
    pub country: String,
    pub lat: f64,
    pub lon: f64,
    /// Itinerary days that call at this port. Used to rank findings by blast radius.
    pub days: u32,
}

#[derive(Serialize, Clone)]
pub struct SnapReport {
    pub name: String,
    pub country: String,
    pub lat: f64,
    pub lon: f64,
    pub days: u32,
    /// Distance from the port to the graph node its routes start from.
    pub snap_km: f64,
    pub snap_lon: f64,
    pub snap_lat: f64,
    /// False when no candidate node had a clear water connector, so the route
    /// starts at the fallback node instead of the port.
    pub connector_clear: bool,
    /// Distance to the geometrically nearest node in ANY component, reachable
    /// or not. When this is far below `snap_km` the port sits on a water body
    /// the router cannot reach, e.g. the Sea of Marmara behind the Bosphorus.
    pub nearest_any_km: f64,
    /// Size of the component holding that nearest node.
    pub nearest_any_comp_size: usize,
    /// True when that node is in the component the router actually routes on.
    pub nearest_any_is_main: bool,
    /// Distance to the nearest point exact ring geometry calls water, found by
    /// scanning outward from the port. `None` means the land polygons show no
    /// water anywhere near this port — a river berth above the coastline, which
    /// no graph resolution can fix. Those need a corridor in waterways.geojson.
    pub exact_water_km: Option<f64>,
}

pub fn load_ports(path: &str) -> Result<Vec<Port>, Box<dyn std::error::Error>> {
    Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
}

pub fn snap_ports(
    graph: &Graph,
    classifier: &LandClassifier,
    ports: &[Port],
    full_index: &RTree<NodeEntry>,
) -> Vec<SnapReport> {
    ports
        .iter()
        .map(|p| {
            let snap = snap_to_water(graph, classifier, p.lon, p.lat);
            let (snap_lon, snap_lat) = (graph.lon(snap.node), graph.lat(snap.node));

            let near_any = full_index.nearest_neighbor(&[p.lon, p.lat]);
            let (nearest_any_km, nearest_any_comp_size, nearest_any_is_main) = match near_any {
                Some(e) => {
                    let comp = graph.component[e.id as usize];
                    (
                        haversine_km(p.lon, p.lat, e.lon, e.lat),
                        graph.comp_sizes.get(comp as usize).copied().unwrap_or(0),
                        comp == graph.main_comp,
                    )
                }
                None => (f64::INFINITY, 0, false),
            };

            SnapReport {
                name: p.name.clone(),
                country: p.country.clone(),
                lat: p.lat,
                lon: p.lon,
                days: p.days,
                snap_km: haversine_km(p.lon, p.lat, snap_lon, snap_lat),
                snap_lon,
                snap_lat,
                connector_clear: snap.connector_clear,
                exact_water_km: scan_for_water(classifier, p.lon, p.lat),
                nearest_any_km,
                nearest_any_comp_size,
                nearest_any_is_main,
            }
        })
        .collect()
}

/// Radius, in km, out to which `scan_for_water` looks.
const WATER_SCAN_MAX_KM: f64 = 8.0;

/// Distance to the nearest point the exact ring geometry calls water, scanning
/// rings of increasing radius around the port. Independent of the graph: it
/// answers "is there water here at all", not "did the quadtree find it".
pub fn scan_for_water(classifier: &LandClassifier, lon: f64, lat: f64) -> Option<f64> {
    let km_per_deg_lat = 111.0;
    let km_per_deg_lon = 111.0 * lat.to_radians().cos().abs().max(0.02);
    let mut r = 0.2;
    while r <= WATER_SCAN_MAX_KM {
        let steps = ((r * 12.0) as usize).clamp(16, 256);
        for i in 0..steps {
            let theta = std::f64::consts::TAU * i as f64 / steps as f64;
            let plon = lon + (r * theta.cos()) / km_per_deg_lon;
            let plat = lat + (r * theta.sin()) / km_per_deg_lat;
            if !classifier.is_land_precise(plon, plat) {
                return Some(r);
            }
        }
        r *= 1.35;
    }
    None
}

/// Percentile of an already-sorted-ascending slice of distances.
pub fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx]
}

/// Snap quality frozen from a known-good run. Ports regress against this.
#[derive(Serialize, Deserialize, Clone)]
pub struct Baseline {
    pub name: String,
    pub country: String,
    pub lat: f64,
    pub lon: f64,
    pub snap_km: f64,
    pub connector_clear: bool,
}

/// Stable identity for a port row. Names alone are not unique: the fixture has
/// two St. John's (AG and CA) and two Sydneys (AU and CA), and nothing stops a
/// future export adding a same-name pair within one country.
pub fn port_key(name: &str, country: &str, lat: f64, lon: f64) -> String {
    format!("{}|{}|{:.4},{:.4}", name, country, lat, lon)
}

impl SnapReport {
    pub fn key(&self) -> String {
        port_key(&self.name, &self.country, self.lat, self.lon)
    }
    pub fn label(&self) -> String {
        format!("{} ({})", self.name, self.country)
    }
}

impl Baseline {
    pub fn key(&self) -> String {
        port_key(&self.name, &self.country, self.lat, self.lon)
    }
}

pub fn to_baseline(reports: &[SnapReport]) -> Vec<Baseline> {
    let mut b: Vec<Baseline> = reports
        .iter()
        .map(|r| Baseline {
            name: r.name.clone(),
            country: r.country.clone(),
            lat: r.lat,
            lon: r.lon,
            snap_km: (r.snap_km * 100.0).round() / 100.0,
            connector_clear: r.connector_clear,
        })
        .collect();
    b.sort_by(|a, b| a.key().cmp(&b.key()));
    b
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::Graph;
    use std::collections::HashMap;
    use std::sync::OnceLock;

    /// How close a port must attach to the graph.
    ///
    /// Unreachable with the depth-16 graph as generated: a cell is ~430m × 305m
    /// at mid latitudes and a node sits at its centre, so the closest any port
    /// currently gets is 47m and only 17 of 604 are inside 100m. Closing this
    /// means injecting the port coordinates themselves as graph nodes during
    /// generation, the way `canals.rs` injects canal waypoints. The test states
    /// the target; `no_port_snap_regressed_against_baseline` is the gate that
    /// stays green meanwhile.
    const SNAP_BUDGET_KM: f64 = 0.1;

    /// Ports whose fixture coordinates are wrong, so no amount of routing work
    /// can reach them. Listed with the evidence, and the tests assert the list
    /// is exact: if one starts passing, the test fails and tells you to delete
    /// the entry. That stops it quietly becoming a dumping ground.
    const KNOWN_BAD_COORDS: &[(&str, &str)] = &[(
        "Coquimbo",
        "fixture has -30.7547,-70.9006 — about 45km inland in the Andes. \
         The port is near -29.95,-71.34. Fix in the ports table, then \
         re-export the fixture.",
    )];

    fn is_known_bad(name: &str) -> bool {
        KNOWN_BAD_COORDS.iter().any(|(n, _)| *n == name)
    }

    /// Fails when a listed port starts working, so the list cannot rot.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn known_bad_coordinate_list_is_current() {
        let fixed: Vec<&str> = KNOWN_BAD_COORDS
            .iter()
            .filter(|(n, _)| {
                reports().iter().any(|r| r.name == *n && r.connector_clear)
            })
            .map(|(n, _)| *n)
            .collect();
        assert!(
            fixed.is_empty(),
            "{:?} now reach the graph — remove them from KNOWN_BAD_COORDS",
            fixed
        );
    }

    /// Allowed drift before a snap counts as a regression. Absorbs float noise
    /// without hiding a real move.
    const REGRESSION_TOLERANCE_KM: f64 = 0.05;

    fn data_dir() -> String {
        std::env::var("SEA_ROUTER_DATA").unwrap_or_else(|_| "../data".into())
    }

    struct World {
        graph: Graph,
        classifier: LandClassifier,
        ports: Vec<Port>,
    }

    /// Loading the graph costs ~30s and 3GB. Share one across every test.
    fn world() -> &'static World {
        static WORLD: OnceLock<World> = OnceLock::new();
        WORLD.get_or_init(|| {
            let dir = data_dir();
            let graph = Graph::load(&format!("{}/graph/sea-graph.json", dir))
                .expect("graph not found — run `generate` first or set SEA_ROUTER_DATA");
            let osm = format!("{}/osm_land_simplified.geojson.json", dir);
            let land = if std::path::Path::new(&osm).exists() {
                osm
            } else {
                format!("{}/ne_10m_land.geojson.json", dir)
            };
            let classifier = LandClassifier::load(&land).expect("land data not found");
            let ports = load_ports("tests/fixtures/ports.json").expect("ports fixture");
            World { graph, classifier, ports }
        })
    }

    fn reports() -> &'static Vec<SnapReport> {
        static REPORTS: OnceLock<Vec<SnapReport>> = OnceLock::new();
        REPORTS.get_or_init(|| {
            let w = world();
            let full = w.graph.build_full_index();
            snap_ports(&w.graph, &w.classifier, &w.ports, &full)
        })
    }

    /// The property that matters for the drawn line: every port must attach to
    /// the graph over open water, so `stitch_endpoints` can put the real port
    /// coordinate on the polyline. A port without a clear connector draws a
    /// route that starts at the fallback node, nowhere near the pin.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn every_port_has_a_clear_connector() {
        let mut bad: Vec<&SnapReport> = reports()
            .iter()
            .filter(|r| !r.connector_clear && !is_known_bad(&r.name))
            .collect();
        bad.sort_by(|a, b| b.days.cmp(&a.days));

        assert!(
            bad.is_empty(),
            "{} of {} ports have no clear water connector, so their routes do not \
             reach the port. Worst by itinerary days:\n{}",
            bad.len(),
            reports().len(),
            format_ports(&bad, 25),
        );
    }

    /// A port should attach to the water next to it, not to the next sea over.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn every_port_snaps_within_budget() {
        let mut bad: Vec<&SnapReport> = reports()
            .iter()
            .filter(|r| r.snap_km > SNAP_BUDGET_KM && !is_known_bad(&r.name))
            .collect();
        bad.sort_by(|a, b| b.snap_km.partial_cmp(&a.snap_km).unwrap());

        assert!(
            bad.is_empty(),
            "{} of {} ports attach further than {:.0}m from their coordinates \
             (worst first):\n{}",
            bad.len(),
            reports().len(),
            SNAP_BUDGET_KM * 1000.0,
            format_ports(&bad, 25),
        );
    }

    /// A port with reachable-looking water closer than the node it snapped to
    /// is sitting behind a strait the quadtree closed off. The Sea of Marmara
    /// (Istanbul) is the clearest case: water 2.5km away, in a 3345-node island
    /// component the router never indexes, so it snaps 187km into the Aegean.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn no_port_sits_beside_stranded_water() {
        let mut bad: Vec<&SnapReport> = reports()
            .iter()
            .filter(|r| !r.nearest_any_is_main && r.nearest_any_km < r.snap_km * 0.5)
            .filter(|r| !is_known_bad(&r.name))
            .collect();
        bad.sort_by(|a, b| b.days.cmp(&a.days));

        assert!(
            bad.is_empty(),
            "{} ports sit beside water the router cannot reach — the connecting \
             strait or channel is narrower than a quadtree cell:\n{}",
            bad.len(),
            format_ports(&bad, 25),
        );
    }

    /// The regression gate. Unlike the three above this is green today: it
    /// freezes the current state and fails the moment a change makes any port
    /// worse. Refresh with `sea-router-rs audit --write-baseline` after a graph
    /// change that improves things, and check the diff.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn no_port_snap_regressed_against_baseline() {
        let raw = std::fs::read_to_string("tests/fixtures/snap-baseline.json")
            .expect("baseline missing — run `sea-router-rs audit --write-baseline`");
        let baseline: Vec<Baseline> = serde_json::from_str(&raw).expect("baseline parse");
        let by_key: HashMap<String, &Baseline> =
            baseline.iter().map(|b| (b.key(), b)).collect();
        assert_eq!(
            by_key.len(),
            baseline.len(),
            "baseline has duplicate port keys — name/country/coords is no longer unique"
        );

        let mut failures: Vec<String> = Vec::new();

        for r in reports() {
            let Some(b) = by_key.get(&r.key()) else {
                failures.push(format!(
                    "  {:<34} not in baseline — regenerate it after changing the ports fixture",
                    r.label()
                ));
                continue;
            };
            if b.connector_clear && !r.connector_clear {
                failures.push(format!(
                    "  {:<34} lost its water connector (was clear)",
                    r.label()
                ));
            }
            // A snap that moved further is only a regression if the connector
            // is still not clear. Snapping past a nearer node to reach one the
            // port can actually see over water is the fix working, not a loss.
            if r.snap_km > b.snap_km + REGRESSION_TOLERANCE_KM && !r.connector_clear {
                failures.push(format!(
                    "  {:<34} snap {:.2}km → {:.2}km (+{:.2}), still no clear connector",
                    r.label(),
                    b.snap_km,
                    r.snap_km,
                    r.snap_km - b.snap_km
                ));
            }
        }

        assert!(
            failures.is_empty(),
            "{} port(s) regressed:\n{}",
            failures.len(),
            failures.join("\n"),
        );
    }

    fn format_ports(ports: &[&SnapReport], limit: usize) -> String {
        let mut out: Vec<String> = ports
            .iter()
            .take(limit)
            .map(|r| {
                format!(
                    "  {:<34} {:>5} days  snap {:>8.2}km  nearest water {:>8.2}km ({})",
                    r.label(),
                    r.days,
                    r.snap_km,
                    r.nearest_any_km,
                    if r.nearest_any_is_main {
                        "routable".to_string()
                    } else {
                        format!("stranded, {} nodes", r.nearest_any_comp_size)
                    },
                )
            })
            .collect();
        if ports.len() > limit {
            out.push(format!("  … and {} more", ports.len() - limit));
        }
        out.join("\n")
    }

    /// Port pairs that exercise the hard cases: a river berth, a sealed
    /// harbour, a fjord head, a canal transit and a long ocean crossing.
    const ROUTE_CASES: &[(&str, &str)] = &[
        ("Hamburg", "Southampton"),
        ("Manaus", "Bridgetown"),
        ("Montreal", "Halifax"),
        ("Istanbul", "Piraeus (Athens)"),
        ("Sydney", "Auckland"),
        ("Venice", "Dubrovnik"),
        ("Geiranger", "Bergen"),
        ("Seattle", "Juneau"),
        ("Miami", "Nassau"),
        ("Southampton", "Barcelona"),
    ];

    fn port_named(name: &str) -> &'static Port {
        world()
            .ports
            .iter()
            .find(|p| p.name == name)
            .unwrap_or_else(|| panic!("port {:?} not in fixture", name))
    }

    /// The line the map draws must start and end at the berth. Anything else
    /// and the route visibly detaches from its pin.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn routes_start_and_end_at_their_ports() {
        let w = world();
        let mut router = crate::router::Router::new(w.graph.node_count);
        let mut bad = Vec::new();

        for (a, b) in ROUTE_CASES {
            let (pa, pb) = (port_named(a), port_named(b));
            let Some(r) = crate::server::route_pipeline(
                &w.graph, &w.classifier, &mut router,
                [pa.lon, pa.lat], [pb.lon, pb.lat], 5.0,
            ) else {
                bad.push(format!("  {} → {}: no route at all", a, b));
                continue;
            };
            let start = *r.smoothed.first().unwrap();
            let end = *r.smoothed.last().unwrap();
            let ds = haversine_km(pa.lon, pa.lat, start[0], start[1]);
            let de = haversine_km(pb.lon, pb.lat, end[0], end[1]);
            if ds > SNAP_BUDGET_KM || de > SNAP_BUDGET_KM {
                bad.push(format!(
                    "  {} → {}: starts {:.0}m from {}, ends {:.0}m from {}",
                    a, b, ds * 1000.0, a, de * 1000.0, b
                ));
            }
        }
        assert!(bad.is_empty(), "{} route(s) do not reach their ports:\n{}", bad.len(), bad.join("\n"));
    }

    /// The smoothed line is what the map renders, and it is built with the
    /// coarse raster. Re-check it against exact geometry: a route that clips a
    /// headland is the most visible defect there is.
    #[test]
    #[ignore = "needs the full routing graph; run with --ignored"]
    fn routes_do_not_cross_land() {
        let w = world();
        let mut router = crate::router::Router::new(w.graph.node_count);
        let mut bad = Vec::new();

        for (a, b) in ROUTE_CASES {
            let (pa, pb) = (port_named(a), port_named(b));
            let Some(r) = crate::server::route_pipeline(
                &w.graph, &w.classifier, &mut router,
                [pa.lon, pa.lat], [pb.lon, pb.lat], 5.0,
            ) else { continue };

            // Ignore the first and last kilometre: berths sit on the shoreline.
            let mut hits = Vec::new();
            for seg in r.smoothed.windows(2) {
                let d = haversine_km(seg[0][0], seg[0][1], seg[1][0], seg[1][1]);
                let steps = ((d / 1.0).ceil() as usize).max(1);
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let lon = seg[0][0] + t * (seg[1][0] - seg[0][0]);
                    let lat = seg[0][1] + t * (seg[1][1] - seg[0][1]);
                    if haversine_km(lon, lat, pa.lon, pa.lat) < 1.5
                        || haversine_km(lon, lat, pb.lon, pb.lat) < 1.5
                    {
                        continue;
                    }
                    if w.classifier.is_land_precise(lon, lat) {
                        hits.push([lon, lat]);
                    }
                }
            }
            if !hits.is_empty() {
                bad.push(format!(
                    "  {} → {}: {} land samples, first at {:.4},{:.4}",
                    a, b, hits.len(), hits[0][0], hits[0][1]
                ));
            }
        }
        assert!(bad.is_empty(), "{} route(s) cross land:\n{}", bad.len(), bad.join("\n"));
    }
}
