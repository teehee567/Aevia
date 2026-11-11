//! Decode diagnostic frames with the firmware's actual wire schema.
use std::io::{self, BufRead};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args().skip(1);
    if let Some(argument) = arguments.next() {
        if !matches!(argument.as_str(), "--describe" | "--build-id") || arguments.next().is_some() {
            return Err("usage: aevia-board-decode [--describe|--build-id]".into());
        }
        if argument == "--build-id" {
            println!("{}", firmware::device::configuration::BUILD_ID);
            return Ok(());
        }
        println!(
            "{}",
            serde_json::to_string(&firmware::protocol::info(false))?
        );
        return Ok(());
    }
    for line in io::stdin().lock().lines() {
        let line = line?;
        let line = line.trim();
        if !line.len().is_multiple_of(2) || !line.is_ascii() {
            return Err("expected an even-length hexadecimal frame".into());
        }
        let bytes = line
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16))
            .collect::<Result<Vec<_>, _>>()?;
        let response = decode(&bytes)?;
        println!("{}", serde_json::to_string(&response)?);
    }
    Ok(())
}

fn decode(bytes: &[u8]) -> Result<firmware::protocol::Response, &'static str> {
    if !bytes.ends_with(&[0]) {
        return Err("expected one complete zero-terminated response");
    }
    let mut frame = firmware::protocol::Frame::<{ firmware::protocol::RESPONSE_FRAME_SIZE }>::new();
    let mut input = bytes;
    let response = frame
        .feed(&mut input)
        .ok_or("incomplete response")?
        .map_err(|_| "invalid response frame")?;
    if !input.is_empty() {
        return Err("expected exactly one response frame");
    }
    Ok(response)
}

#[cfg(test)]
mod tests {
    use super::decode;

    #[test]
    fn rejects_missing_delimiter_multiple_frames_and_trailing_postcard_bytes() {
        // System::Ok is Postcard [0, 0].
        assert!(decode(&[1, 1, 1, 0]).is_ok());
        assert!(decode(&[1, 1, 1]).is_err());
        assert!(decode(&[1, 1, 1, 0, 1, 1, 1, 0]).is_err());
        assert!(decode(&[1, 1, 2, 1, 0]).is_err());
    }

    #[test]
    fn decodes_discovery_using_the_same_schema_as_the_firmware() {
        let expected = firmware::protocol::info(false);
        let response = firmware::protocol::Response::System(
            firmware::commands::Response::ProtocolInfo(expected),
        );
        let mut bytes = [0; firmware::protocol::RESPONSE_FRAME_SIZE];
        let bytes = postcard::to_slice_cobs(&response, &mut bytes).unwrap();
        let firmware::protocol::Response::System(firmware::commands::Response::ProtocolInfo(
            actual,
        )) = decode(bytes).unwrap()
        else {
            panic!()
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn charge_readback_decodes_to_named_fields_without_claiming_active_charge() {
        use firmware::device::interface::{DeviceResponse, PowerChargeStatus};
        let expected = PowerChargeStatus {
            updated_us: Some(5_000_000),
            verified: true,
            charging_enabled: Some(true),
            charge_current_ma: Some(400),
            charge_voltage_mv: Some(4100),
            input_current_limit_ma: Some(500),
            charger_status0: Some(0),
            charger_status1: Some(0),
            fault_status: Some(0),
            precharge_current_ma: Some(80),
            termination_current_ma: Some(40),
            charger_control0: Some(0),
            charger_control1: Some(0x20),
            ntc_control0: Some(0),
        };
        let response =
            firmware::protocol::Response::Device(DeviceResponse::PowerChargeStatus(expected));
        let mut bytes = [0; 512];
        let bytes = postcard::to_slice_cobs(&response, &mut bytes).unwrap();
        let actual = decode(bytes).unwrap();
        let json = serde_json::to_value(actual).unwrap();
        let status = &json["Device"]["PowerChargeStatus"];
        assert_eq!(status["charge_current_ma"], 400);
        assert_eq!(status["charge_voltage_mv"], 4100);
        assert_eq!(status["charger_status1"], 0);
        assert_eq!(status["verified"], true);
        assert!(status.get("charging_active").is_none());
    }

    #[test]
    fn recording_download_decodes_bounded_bytes_without_changing_existing_envelopes() {
        use firmware::device::recording::api;
        let expected = api::Response {
            request_id: 42,
            reply: api::Reply::Data(api::Chunk {
                session: 7,
                offset: 256,
                bytes: api::Bytes::new(&[0xa5; api::MAX_READ_BYTES]).unwrap(),
                crc32: 123,
            }),
        };
        let response = firmware::protocol::Response::Recording(expected);
        let mut bytes = [0; firmware::protocol::RESPONSE_FRAME_SIZE];
        let bytes = postcard::to_slice_cobs(&response, &mut bytes).unwrap();
        let firmware::protocol::Response::Recording(actual) = decode(bytes).unwrap() else {
            panic!()
        };
        assert_eq!(actual, expected);
        let json = serde_json::to_value(firmware::protocol::Response::Recording(actual)).unwrap();
        assert_eq!(
            json["Recording"]["reply"]["Data"]["bytes"]
                .as_array()
                .unwrap()
                .len(),
            api::MAX_READ_BYTES
        );
    }
}
