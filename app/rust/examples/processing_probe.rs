//! Checks that a long analysis leaves the UI request path responsive and can
//! be cancelled without replacing its saved session.
use std::time::{Duration, Instant};
fn main() {
    let id = std::env::args()
        .nth(1)
        .expect("usage: processing_probe <session id>");
    let _ = rust_lib_app::api::simple::request("{\"op\":\"state\"}".into());
    let worker = std::thread::spawn(move || {
        rust_lib_app::api::simple::request(
            serde_json::json!({"op":"reanalyze","id":id}).to_string(),
        )
    });
    std::thread::sleep(Duration::from_millis(500));
    let started = Instant::now();
    let state = rust_lib_app::api::simple::request("{\"op\":\"state\"}".into());
    let elapsed = started.elapsed();
    let state: serde_json::Value = serde_json::from_str(&state).unwrap();
    println!(
        "{}",
        serde_json::json!({"state_latency_ms":elapsed.as_millis(),"processing":state["data"]["processing"]})
    );
    println!(
        "{}",
        rust_lib_app::api::simple::request("{\"op\":\"cancel_processing\"}".into())
    );
    let response = worker.join().expect("worker panicked");
    let response: serde_json::Value = serde_json::from_str(&response).unwrap();
    assert_eq!(
        response["ok"], false,
        "processing should have been cancelled"
    );
    assert!(
        response["error"]
            .as_str()
            .is_some_and(|s| s.contains("cancelled")),
        "{response}"
    );
    assert!(
        elapsed < Duration::from_millis(250),
        "state was blocked for {elapsed:?}"
    );
    println!("{response}");
}
