use std::collections::VecDeque;

use crate::audit::Port;
use crate::graph::Graph;
use crate::land::LandClassifier;
use crate::router::{haversine_km, snap_to_water};

/// Grid step for the channel search, in km. Fine enough to fit through a
/// ~700m strait, coarse enough that a 120km box stays cheap.
const STEP_KM: f64 = 0.15;
/// How far out to look before giving up on a port. Sognefjord alone runs
/// ~200km from its head to the sea, so this has to clear that.
const MAX_RADIUS_KM: f64 = 420.0;
/// Berths sit on the shoreline and often inside a harbour the polygons have
/// sealed, so the first stretch out of the port has to ignore land or the
/// search dies on its first step.
const SEED_RADIUS_KM: f64 = 1.5;
/// A cell counts as an exit once a main-component node is this close and the
/// straight line to it stays in water.
const EXIT_LINK_KM: f64 = 2.0;
/// Radius that must be clear water in every direction for a point to count as
/// open sea. `reconnect_islands` bridges stranded fjord cells into the main
/// component, so "a main-component node is nearby" is true well inside a fjord
/// and on its own stops the search halfway up. A fjord is a few km across at
/// most; this ring only clears once the search is genuinely outside one.
const OPEN_WATER_RADIUS_KM: f64 = 4.0;

/// A channel found by searching exact ring geometry, ready to be written out
/// as a corridor.
pub struct Channel {
    pub port: String,
    pub points: Vec<[f64; 2]>,
}

/// Search for a navigable path from a port out to water the raster can see.
///
/// Fjords and straits are real water in the land polygons but narrower than the
/// 0.02° raster, so the quadtree never forms cells in them and the port is left
/// attached to nothing. This walks the exact geometry outward until it reaches
/// raster-visible water, which is where ordinary graph cells already exist. The
/// resulting line is written to waterways.geojson like any other corridor, and
/// from then on it is carved into the raster and needs no special handling.
///
/// Returns `None` when no path exists inside `MAX_RADIUS_KM` — a port that is
/// genuinely landlocked in this dataset, or one whose coordinates are wrong.
pub fn find_channel(graph: &Graph, classifier: &LandClassifier, port: &Port) -> Option<Channel> {
    let km_per_deg_lat = 111.0;
    let km_per_deg_lon = 111.0 * port.lat.to_radians().cos().abs().max(0.02);

    let half = (MAX_RADIUS_KM / STEP_KM).ceil() as i32;
    let side = (half * 2 + 1) as usize;
    let idx = |gx: i32, gy: i32| -> usize { ((gy + half) as usize) * side + (gx + half) as usize };
    let to_lonlat = |gx: i32, gy: i32| -> [f64; 2] {
        [
            port.lon + (gx as f64 * STEP_KM) / km_per_deg_lon,
            port.lat + (gy as f64 * STEP_KM) / km_per_deg_lat,
        ]
    };

    let mut came: Vec<i32> = vec![i32::MIN; side * side];
    let mut queue = VecDeque::new();
    came[idx(0, 0)] = i32::MAX; // root marker
    queue.push_back((0i32, 0i32));

    // Seed outward through the shoreline so a berth inside a sealed harbour
    // still has somewhere to go.
    let seed = (SEED_RADIUS_KM / STEP_KM).ceil() as i32;
    for gy in -seed..=seed {
        for gx in -seed..=seed {
            if gx == 0 && gy == 0 { continue; }
            if ((gx * gx + gy * gy) as f64).sqrt() * STEP_KM > SEED_RADIUS_KM { continue; }
            let i = idx(gx, gy);
            if came[i] == i32::MIN {
                came[i] = idx(0, 0) as i32;
                queue.push_back((gx, gy));
            }
        }
    }

    while let Some((gx, gy)) = queue.pop_front() {
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
            let (nx, ny) = (gx + dx, gy + dy);
            if nx.abs() > half || ny.abs() > half {
                continue;
            }
            let i = idx(nx, ny);
            if came[i] != i32::MIN {
                continue;
            }
            let p = to_lonlat(nx, ny);
            if classifier.is_land_precise(p[0], p[1]) {
                came[i] = i32::MAX - 1; // visited, impassable
                continue;
            }
            came[i] = idx(gx, gy) as i32;

            // An exit is water the router can actually route from: a node in
            // the main component, close by, with a clear line to it. Merely
            // reaching raster-visible water is not enough — a stranded fjord is
            // full of it, and every cell belongs to the same unreachable island.
            if !classifier.is_land(p[0], p[1]) && is_exit(graph, classifier, p) {
                let mut path = vec![p];
                let mut cur = i;
                while came[cur] != i32::MAX {
                    let prev = came[cur] as usize;
                    let px = (prev % side) as i32 - half;
                    let py = (prev / side) as i32 - half;
                    path.push(to_lonlat(px, py));
                    cur = prev;
                }
                path.push([port.lon, port.lat]);
                path.reverse();
                return Some(Channel { port: port.name.clone(), points: thin(&path) });
            }
            queue.push_back((nx, ny));
        }
    }
    None
}

/// True if the router could start a route from here: the nearest main-component
/// node is close and the port can see it over water.
fn is_exit(graph: &Graph, classifier: &LandClassifier, p: [f64; 2]) -> bool {
    if !is_open_water(classifier, p) {
        return false;
    }
    let snap = snap_to_water(graph, classifier, p[0], p[1]);
    if !snap.connector_clear {
        return false;
    }
    haversine_km(p[0], p[1], graph.lon(snap.node), graph.lat(snap.node)) <= EXIT_LINK_KM
}

/// True when every compass direction is still water `OPEN_WATER_RADIUS_KM` out.
fn is_open_water(classifier: &LandClassifier, p: [f64; 2]) -> bool {
    let dlat = OPEN_WATER_RADIUS_KM / 111.0;
    let dlon = dlat / p[1].to_radians().cos().abs().max(0.05);
    for i in 0..8 {
        let th = std::f64::consts::TAU * i as f64 / 8.0;
        if classifier.is_land(p[0] + dlon * th.cos(), p[1] + dlat * th.sin()) {
            return false;
        }
    }
    true
}

/// Douglas-Peucker, so a 150m-step BFS path becomes a reviewable polyline.
fn thin(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    const TOL_DEG: f64 = 0.002;
    let n = points.len();
    if n <= 2 {
        return points.to_vec();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((si, ei)) = stack.pop() {
        if ei <= si + 1 {
            continue;
        }
        let (s, e) = (points[si], points[ei]);
        let (dx, dy) = (e[0] - s[0], e[1] - s[1]);
        let len = (dx * dx + dy * dy).sqrt();
        let (mut best_d, mut best_i) = (0.0f64, si);
        for i in (si + 1)..ei {
            let p = points[i];
            let d = if len > 0.0 {
                (dy * p[0] - dx * p[1] + e[0] * s[1] - e[1] * s[0]).abs() / len
            } else {
                ((p[0] - s[0]).powi(2) + (p[1] - s[1]).powi(2)).sqrt()
            };
            if d > best_d {
                best_d = d;
                best_i = i;
            }
        }
        if best_d > TOL_DEG {
            keep[best_i] = true;
            stack.push((si, best_i));
            stack.push((best_i, ei));
        }
    }
    points.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}
