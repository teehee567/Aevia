//! Line-oriented native smoke-test client. Feed one JSON operation per line.
//! A {"sleep_ms":500} line waits while the acquisition worker keeps collecting.
use std::io::{self, BufRead};
fn main() {
    for line in io::stdin().lock().lines() {
        let line = match line {
            Ok(line) => line,
            Err(error) => {
                eprintln!("{error}");
                break;
            }
        };
        if line.trim().is_empty() {
            continue;
        }
        if let Ok(value) = serde_json::from_str::<serde_json::Value>(&line) {
            if let Some(delay) = value.get("sleep_ms").and_then(serde_json::Value::as_u64) {
                std::thread::sleep(std::time::Duration::from_millis(delay.min(60_000)));
                continue;
            }
        }
        let response = rust_lib_app::api::simple::request(line);
        if std::env::var_os("AEVIA_CLI_SUMMARY").is_some() {
            let mut value: serde_json::Value = serde_json::from_str(&response).expect("API JSON");
            if let Some(data) = value
                .get_mut("data")
                .and_then(serde_json::Value::as_object_mut)
            {
                data.remove("samples");
                data.remove("raw_records");
            }
            println!("{value}");
        } else {
            println!("{response}");
        }
    }
}
