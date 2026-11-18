fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: inspect_capture <file.aev>");
    println!(
        "{}",
        rust_lib_app::api::simple::request(
            serde_json::json!({"op":"inspect","path":path}).to_string()
        )
    );
}
