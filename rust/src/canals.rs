/// Manual canal/strait waypoints that must appear in the graph.
/// These are man-made or narrow natural waterways that get incorrectly
/// classified as land by the NE polygon data.

pub struct CanalPassage {
    pub name: &'static str,
    pub waypoints: &'static [[f64; 2]], // [lon, lat] pairs
}

/// Canals and straits that need manual injection.
/// Waypoints form a path through each waterway.
pub static CANALS: &[CanalPassage] = &[
    CanalPassage {
        name: "Suez Canal",
        // 28 waypoints derived from OpenStreetMap canal centerline,
        // simplified from 92 original nodes with Douglas-Peucker ε=0.001
        waypoints: &[
            [32.3263, 31.2757],  // Port Said entrance
            [32.3067, 31.2505],  // Port Said approach channel
            [32.3047, 31.2402],
            [32.3043, 31.2200],  // South of Port Said
            [32.3177, 30.8114],  // North of Lake Timsah
            [32.3353, 30.7482],  // Lake Timsah / Ismailia
            [32.3437, 30.7128],
            [32.3440, 30.7050],  // South of Lake Timsah
            [32.3243, 30.6200],  // Entering Great Bitter Lake
            [32.3048, 30.5809],  // Great Bitter Lake (west shore)
            [32.3039, 30.5656],  // Great Bitter Lake center
            [32.3088, 30.5496],
            [32.3342, 30.5181],  // Great Bitter Lake (south)
            [32.3390, 30.5061],
            [32.3500, 30.4522],  // Little Bitter Lake
            [32.3578, 30.4352],
            [32.3729, 30.3606],  // South of Little Bitter Lake
            [32.4428, 30.2827],  // Curve eastward
            [32.5292, 30.2532],  // Southern canal section
            [32.5387, 30.2429],
            [32.5654, 30.2010],  // Approaching Gulf of Suez
            [32.5685, 30.1865],
            [32.5731, 30.0537],  // South of canal
            [32.5868, 29.9728],  // Port Tewfik approach
            [32.5841, 29.9576],
            [32.5805, 29.9506],
            [32.5759, 29.9436],
            [32.5607, 29.9303],  // Suez / Port Tewfik entrance
        ],
    },
    CanalPassage {
        name: "Panama Canal",
        waypoints: &[
            [-79.915, 9.390],  // Atlantic entrance (Colón)
            [-79.900, 9.350],
            [-79.880, 9.300],
            [-79.860, 9.250],
            [-79.840, 9.200],
            [-79.820, 9.170],
            [-79.780, 9.140],
            [-79.740, 9.100],
            [-79.700, 9.060],
            [-79.660, 9.020],
            [-79.620, 8.980],
            [-79.580, 8.960],
            [-79.540, 8.940],
            [-79.530, 8.900],  // Pacific entrance (Balboa)
        ],
    },
    CanalPassage {
        name: "Kiel Canal",
        waypoints: &[
            [10.155, 54.365],  // Brunsbüttel (Elbe/North Sea)
            [9.940, 54.330],
            [9.750, 54.320],
            [9.550, 54.310],
            [9.350, 54.310],
            [9.160, 54.330],
            [8.970, 54.340],
            [8.780, 54.350],  // Kiel (Baltic)
        ],
    },
];
