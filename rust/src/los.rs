use crate::land::LandClassifier;

/// Line-of-sight path simplification using forward greedy scan
/// with segmented bbox rejection.
pub fn simplify(
    path: &[[f64; 2]],
    classifier: &LandClassifier,
    interval_km: f64,
) -> Vec<[f64; 2]> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let mut result = vec![path[0]];
    let mut current = 0;

    while current < path.len() - 1 {
        let mut best = current + 1;

        for j in (current + 2)..path.len() {
            if is_line_in_water(&path[current], &path[j], classifier, interval_km) {
                best = j;
            } else {
                break;
            }
        }

        result.push(path[best]);
        current = best;
    }

    result
}

/// Check if a straight line between two points stays in water.
/// Splits long lines into ~100km segments with bbox rejection.
fn is_line_in_water(
    from: &[f64; 2],
    to: &[f64; 2],
    classifier: &LandClassifier,
    interval_km: f64,
) -> bool {
    let dlon = to[0] - from[0];
    let dlat = to[1] - from[1];
    let approx_km = (dlon * dlon + dlat * dlat).sqrt() * 111.0;

    if approx_km < 200.0 {
        // Short line: bbox check then sample
        let (min_lon, max_lon) = if from[0] < to[0] { (from[0], to[0]) } else { (to[0], from[0]) };
        let (min_lat, max_lat) = if from[1] < to[1] { (from[1], to[1]) } else { (to[1], from[1]) };

        if !classifier.overlaps_land(min_lon, min_lat, max_lon, max_lat) {
            return true;
        }
        return sample_line(from[0], from[1], dlon, dlat, approx_km, classifier, interval_km);
    }

    // Long line: segment into ~100km chunks
    let num_segments = (approx_km / 100.0).ceil() as usize;

    for s in 0..num_segments {
        let t0 = s as f64 / num_segments as f64;
        let t1 = ((s + 1) as f64 / num_segments as f64).min(1.0);

        let s_lon1 = from[0] + t0 * dlon;
        let s_lat1 = from[1] + t0 * dlat;
        let s_lon2 = from[0] + t1 * dlon;
        let s_lat2 = from[1] + t1 * dlat;

        let (min_lon, max_lon) = if s_lon1 < s_lon2 { (s_lon1, s_lon2) } else { (s_lon2, s_lon1) };
        let (min_lat, max_lat) = if s_lat1 < s_lat2 { (s_lat1, s_lat2) } else { (s_lat2, s_lat1) };

        if !classifier.overlaps_land(min_lon, min_lat, max_lon, max_lat) {
            continue;
        }

        let seg_dlon = s_lon2 - s_lon1;
        let seg_dlat = s_lat2 - s_lat1;
        let seg_km = (seg_dlon * seg_dlon + seg_dlat * seg_dlat).sqrt() * 111.0;

        if !sample_line(s_lon1, s_lat1, seg_dlon, seg_dlat, seg_km, classifier, interval_km) {
            return false;
        }
    }

    true
}

/// Sample points along a line segment checking for land.
/// Also checks ±3km perpendicular band to reject lines that skim
/// too close to coastlines.
#[inline]
fn sample_line(
    lon1: f64, lat1: f64,
    dlon: f64, dlat: f64,
    approx_km: f64,
    classifier: &LandClassifier,
    interval_km: f64,
) -> bool {
    let n = (approx_km / interval_km).ceil().max(2.0) as usize;

    // Perpendicular offset: ~3km in degrees (0.027° ≈ 3km)
    let line_len = (dlon * dlon + dlat * dlat).sqrt();
    let (perp_lon, perp_lat) = if line_len > 1e-10 {
        let offset = 0.027;
        (-dlat / line_len * offset, dlon / line_len * offset)
    } else {
        (0.0, 0.0)
    };

    for i in 1..n {
        let t = i as f64 / n as f64;
        let cx = lon1 + t * dlon;
        let cy = lat1 + t * dlat;

        if classifier.is_land(cx, cy)
            || classifier.is_land(cx + perp_lon, cy + perp_lat)
            || classifier.is_land(cx - perp_lon, cy - perp_lat)
        {
            return false;
        }
    }
    true
}
