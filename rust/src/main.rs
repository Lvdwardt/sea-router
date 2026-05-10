mod graph;
mod land;
mod router;
mod los;
mod server;
mod generate;
mod graph_builder;
mod canals;

use std::sync::Arc;
use std::time::Instant;

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();

    let command = args.get(1).map(|s| s.as_str()).unwrap_or("serve");

    match command {
        "generate" => run_generate(&args).await,
        "serve" => run_server(&args).await,
        _ => {
            println!("🌊 Sea Router (Rust)");
            println!();
            println!("Usage:");
            println!("  sea-router-rs generate [depth] [data-dir]  Build the sea routing graph");
            println!("  sea-router-rs serve [data-dir]             Start the HTTP server");
            println!();
            println!("Defaults: depth=16, data-dir=../data");
        }
    }
}

async fn run_generate(args: &[String]) {
    let max_depth: u8 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(16);
    let data_dir = args.get(3).map(|s| s.as_str()).unwrap_or("../data");

    let t0 = Instant::now();
    println!("\n🌊 Sea Router — Graph Generation (max depth: {})\n", max_depth);

    // Load land data
    println!("Step 1: Loading land data...");
    let land_path = format!("{}/osm_land_simplified.geojson.json", data_dir);
    let land_path = if std::path::Path::new(&land_path).exists() {
        land_path
    } else {
        format!("{}/ne_10m_land.geojson.json", data_dir)
    };
    let classifier = land::LandClassifier::load(&land_path).expect("Failed to load land data");
    println!("  {} land rings loaded\n", classifier.ring_count());

    // Build quadtree
    println!("Step 2: Building quadtree...");
    let mut root = generate::build_quadtree(&classifier, max_depth);
    let (water, land) = generate::count_leaves(&root);
    println!("  Water leaves: {}", water);
    println!("  Land leaves: {}\n", land);

    // Coarsen
    println!("Step 3: Coarsening open water...");
    generate::coarsen(&mut root);
    let (water_after, _) = generate::count_leaves(&root);
    println!("  Water leaves after: {}\n", water_after);

    // Build graph
    println!("Step 4: Building adjacency graph...");
    let leaves = generate::collect_water_leaves(&root);
    let graph_output = graph_builder::build_graph(&leaves, &classifier);

    // Save
    println!("\nStep 5: Saving graph...");
    let output_path = format!("{}/graph/sea-graph.json", data_dir);

    // Ensure directory exists
    if let Some(parent) = std::path::Path::new(&output_path).parent() {
        std::fs::create_dir_all(parent).ok();
    }

    let data = serde_json::json!({
        "nodeCount": graph_output.node_count,
        "edgeCount": graph_output.edge_count,
        "nodes": graph_output.nodes,
        "edges": graph_output.edges,
    });

    std::fs::write(&output_path, serde_json::to_string(&data).unwrap())
        .expect("Failed to write graph");

    let size_mb = std::fs::metadata(&output_path)
        .map(|m| m.len() as f64 / 1_048_576.0)
        .unwrap_or(0.0);

    println!("  Saved to {} ({:.1}MB)", output_path, size_mb);

    let elapsed = t0.elapsed().as_secs_f64();
    println!(
        "\n✅ Done in {:.1}s — {} nodes, {} edges",
        elapsed, graph_output.node_count, graph_output.edge_count
    );
}

async fn run_server(args: &[String]) {
    let t0 = Instant::now();
    println!("🌊 Sea Router (Rust) — Loading...");

    let data_dir = args.get(2).map(|s| s.as_str()).unwrap_or("../data");

    // Load graph
    let graph_path = format!("{}/graph/sea-graph.json", data_dir);
    let graph = graph::Graph::load(&graph_path).expect("Failed to load graph");
    println!(
        "  {} nodes, {} edges loaded in {}ms",
        graph.node_count, graph.edge_count,
        t0.elapsed().as_millis()
    );

    // Load land classifier
    let land_path = format!("{}/osm_land_simplified.geojson.json", data_dir);
    let land_path = if std::path::Path::new(&land_path).exists() {
        land_path
    } else {
        format!("{}/ne_10m_land.geojson.json", data_dir)
    };
    let classifier = land::LandClassifier::load(&land_path).expect("Failed to load land data");
    println!(
        "  {} land rings loaded in {}ms",
        classifier.ring_count(),
        t0.elapsed().as_millis()
    );

    let viewer_path = format!("{}/../viewer.html", data_dir);
    let state = Arc::new(server::AppState { graph, classifier, viewer_path });

    println!(
        "\n🚀 API running at http://localhost:3001 (loaded in {}ms)",
        t0.elapsed().as_millis()
    );

    server::run(state).await;
}
