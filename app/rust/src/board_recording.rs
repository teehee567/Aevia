use crate::format::{self, PAGE_BYTES, RECORD_HEADER, RECORD_PAYLOAD};
use serde_json::{json, Value};

// Inspect only the edges of a file. Records can straddle pages, including
// the close record when the firmware flushes a short or empty final page.
pub(crate) fn preview(
    id: u64,
    sectors: u64,
    closed: bool,
    mut read: impl FnMut(u64) -> Result<[u8; PAGE_BYTES], String>,
) -> Result<Value, String> {
    let mut nonce = None;
    let mut payload = |index| {
        let page = read(index)?;
        let info = format::inspect(&page)
            .filter(|p| u64::from(p.session) == id && u64::from(p.index) == index)
            .ok_or("Recording preview failed page verification")?;
        if info.nonce != *nonce.get_or_insert(info.nonce)
            || info.closed != (index + 1 == sectors && closed)
        {
            return Err("Recording changed while reading its preview".to_owned());
        }
        Ok(page[28..28 + usize::from(info.payload_len)].to_vec())
    };
    let mut head = payload(0)?;
    if sectors > 1 {
        head.extend(payload(1)?);
    }
    let mut records = head.as_slice();
    let mut metadata = serde_json::Map::new();
    let mut start_ns = None;
    while records.len() >= RECORD_HEADER {
        let len = usize::from(u16::from_le_bytes([records[2], records[3]]));
        if len > RECORD_PAYLOAD || records[1] != 1 || records[20..24] != [0; 4] {
            return Err("Invalid recording preview header".into());
        }
        let end = RECORD_HEADER + len;
        if records.len() < end + 4 {
            break;
        }
        let record = &records[..end + 4];
        verify_record(record)?;
        start_ns.get_or_insert_with(|| timestamp(record));
        if record[0] == 1 {
            if let Value::Object(fields) = serde_json::from_slice(&record[RECORD_HEADER..end])
                .map_err(|_| "Invalid recording metadata")?
            {
                metadata.extend(fields);
            }
        }
        records = &records[end + 4..];
    }
    let mut result = json!({"id":id, "metadata":metadata,
        "start_uptime_s":start_ns.map(|t| t as f64 / 1e9)});
    if closed {
        let mut tail = if sectors <= 2 {
            head.clone()
        } else {
            payload(sectors - 1)?
        };
        const CLOSE_LEN: usize = RECORD_HEADER + 4 + 4;
        if tail.len() < CLOSE_LEN && sectors > 2 {
            let mut previous = if sectors == 3 {
                head
            } else {
                payload(sectors - 2)?
            };
            previous.append(&mut tail);
            tail = previous;
        }
        if let Some(end) = tail.last_chunk::<CLOSE_LEN>() {
            if end[..4] == [22, 1, 4, 0] && end[20..24] == [0; 4] {
                verify_record(end)?;
                let stop_ns = timestamp(end);
                result["duration_s"] = json!(start_ns
                    .and_then(|t| stop_ns.checked_sub(t))
                    .map(|d| d as f64 / 1e9));
                result["dropped"] = json!(u32::from_le_bytes(end[24..28].try_into().unwrap()));
            }
        }
    }
    Ok(result)
}

fn timestamp(record: &[u8]) -> u64 {
    u64::from_le_bytes(record[12..20].try_into().unwrap())
}

fn verify_record(record: &[u8]) -> Result<(), String> {
    let (body, crc) = record.split_at(record.len() - 4);
    if format::crc32(body).to_le_bytes() != crc {
        return Err("Recording preview failed record verification".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::{PageBuilder, Record, RECORD_BYTES};

    #[test]
    fn reads_only_bookends_and_handles_a_split_close_record() {
        let mut bytes = Vec::new();
        let mut append = |kind, seq, time, payload: &[u8]| {
            let mut buf = [0; RECORD_BYTES];
            let n = Record::new(kind, seq, time, payload)
                .unwrap()
                .encode(&mut buf);
            bytes.extend_from_slice(&buf[..n]);
        };
        append(1, 0, 1_000_000_000, br#"{"imu_kind":23}"#);
        for seq in 1..12 {
            append(23, seq, 2_000_000_000, &[0; 200]);
        }
        append(22, 12, 13_000_000_000, &7u32.to_le_bytes());
        let mut pages = vec![];
        let tail = bytes.split_off(bytes.len() - 3);
        for chunk in bytes.chunks(480).chain([tail.as_slice()]) {
            let mut page = PageBuilder::new();
            page.append(chunk);
            pages.push(*page.finish(7, pages.len() as u32, 11, chunk.len() == 3));
        }
        let mut reads = vec![];
        let count = pages.len() as u64;
        let result = preview(7, count, true, |index| {
            reads.push(index);
            Ok(pages[index as usize])
        })
        .unwrap();
        assert_eq!(reads, [0, 1, count - 1, count - 2]);
        assert_eq!(result["duration_s"], 12.0);
        assert_eq!(result["dropped"], 7);
        assert_eq!(result["metadata"]["imu_kind"], 23);
        // An interrupted recording has no close-time metrics.
        let partial = preview(7, count - 1, false, |i| Ok(pages[i as usize])).unwrap();
        assert!(partial["duration_s"].is_null() && partial["dropped"].is_null());
        pages[1][20] ^= 1;
        let crc = format::crc32(&pages[1][..508]);
        pages[1][508..].copy_from_slice(&crc.to_le_bytes());
        assert!(preview(7, count, true, |i| Ok(pages[i as usize])).is_err());
    }
}
