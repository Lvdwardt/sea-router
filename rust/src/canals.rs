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
        // Atlantic (Colon/Cristobal) -> Gatun Lake -> Culebra Cut -> Pacific (Balboa)
        // Canal runs NW->SE, ~80km. Coordinates verified against ACP charts.
        waypoints: &[
            [-79.917,  9.383],  // Atlantic entrance -- Limon Bay breakwater (Colon)
            [-79.913,  9.360],  // Approach to Gatun Locks
            [-79.907,  9.280],  // Gatun Locks
            [-79.870,  9.210],  // Gatun Lake -- northern arm
            [-79.820,  9.190],  // Gatun Lake -- central
            [-79.760,  9.160],  // Gatun Lake -- Barro Colorado Island
            [-79.710,  9.130],  // Gatun Lake -- southern arm
            [-79.680,  9.090],  // Gamboa -- Culebra Cut entry
            [-79.640,  9.050],  // Culebra Cut (Gaillard Cut)
            [-79.600,  9.020],  // Pedro Miguel Locks
            [-79.580,  8.990],  // Miraflores Lake
            [-79.560,  8.960],  // Miraflores Locks
            [-79.540,  8.940],  // Balboa approach
            [-79.530,  8.900],  // Pacific entrance -- Flamenco Island (Balboa)
        ],
    },
    CanalPassage {
        name: "Kiel Canal",
        // Nord-Ostsee-Kanal: Brunsbuttel (Elbe/North Sea) -> Kiel-Holtenau (Baltic).
        // ~98 km long, runs roughly W->E through Schleswig-Holstein, Germany.
        // Previous data was completely wrong (reversed direction, ran through Denmark).
        // Corrected from official BSH charts and cruiserswiki coordinates:
        //   Brunsbuttel: 53deg53.2'N 09deg07.8'E  Kiel-Holtenau: 54deg21.5'N 10deg09.65'E
        waypoints: &[
            [ 9.130, 53.887],  // Brunsbuttel locks -- North Sea / Elbe entrance
            [ 9.195, 53.900],  // Brunsbuttel -- east of locks
            [ 9.320, 53.918],  // Kudensee area
            [ 9.480, 53.940],  // Hohenhorn
            [ 9.620, 53.965],  // Breiholz (mid-canal VTS handover)
            [ 9.730, 53.985],  // Rendsburg approaches
            [ 9.820, 54.085],  // Rendsburg -- canal bends northeast
            [ 9.930, 54.175],  // Osterronfeld
            [10.025, 54.265],  // Flemhude
            [10.095, 54.330],  // Levensau
            [10.163, 54.374],  // Kiel-Holtenau locks -- Baltic entrance
        ],
    },
    CanalPassage {
        name: "Corinth Canal",
        // 6.34 km, connects Gulf of Corinth (NW) to Saronic Gulf (SE), Greece.
        // Important for cruise routes between Adriatic/Ionian and Aegean seas.
        // NW entrance: 37deg56.5'N 22deg57.2'E  SE entrance: 37deg55.0'N 23deg00.2'E
        waypoints: &[
            [22.953, 37.941],  // NW entrance -- Poseidonia (Gulf of Corinth)
            [22.966, 37.938],  // Canal western section
            [22.984, 37.935],  // Canal center
            [23.003, 37.918],  // SE entrance -- Isthmia (Saronic Gulf)
        ],
    },
];
