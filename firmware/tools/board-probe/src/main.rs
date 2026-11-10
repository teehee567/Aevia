//! Decode diagnostic frames with the firmware's actual wire schema.
use std::io::{self, BufRead};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    for line in io::stdin().lock().lines() {
        let line = line?;
        let line = line.trim();
        if !line.len().is_multiple_of(2) || !line.is_ascii() {
            return Err("expected an even-length hexadecimal frame".into());
        }
        let mut bytes = line
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16))
            .collect::<Result<Vec<_>, _>>()?;
        let response: firmware::protocol::Response = postcard::from_bytes_cobs(&mut bytes)?;
        println!("{}", serde_json::to_string(&response)?);
    }
    Ok(())
}
