//! Streaming host evidence uses the same checksummed AEVLOG1 format as SD.
use crate::format::{PageBuilder, Record, RECORD_BYTES};
use std::{
    fs::{File, OpenOptions},
    io::{BufWriter, Write},
    path::Path,
};

pub(crate) struct Writer {
    file: BufWriter<File>,
    page: PageBuilder,
    page_used: usize,
    index: u32,
    sequence: u64,
    nonce: u64,
    limit: u64,
    closed: bool,
}

impl Writer {
    pub(crate) fn create(path: &Path, nonce: u64, limit: u64) -> Result<Self, String> {
        let file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        Ok(Self {
            file: BufWriter::with_capacity(64 * 1024, file),
            page: PageBuilder::new(),
            page_used: 0,
            index: 0,
            sequence: 0,
            nonce,
            limit,
            closed: false,
        })
    }
    pub(crate) fn append(&mut self, kind: u8, at_ns: u64, payload: &[u8]) -> Result<(), String> {
        if self.closed {
            return Err("Evidence already closed".into());
        }
        let record = Record::new(kind, self.sequence, at_ns, payload)
            .ok_or("Oversized binary evidence record")?;
        let mut bytes = [0; RECORD_BYTES];
        let len = record.encode(&mut bytes);
        // Reserve the close marker even if a periodic sync flushes the partial
        // data page. Closing exactly on a page boundary also writes a final
        // empty closed page, so use floor+one for that specific operation.
        let needed = if kind == 22 {
            (self.page_used + len) / 480 + 1
        } else {
            (self.page_used + len).div_ceil(480) + 1
        };
        if (u64::from(self.index) + needed as u64) * 512 > self.limit {
            return Err("Binary evidence capacity reached; start another recording".into());
        }
        let mut at = 0;
        while at < len {
            let added = self.page.append(&bytes[at..len]);
            at += added;
            self.page_used += added;
            if self.page.is_full() {
                self.flush_page(false)?;
            }
        }
        self.sequence += 1;
        Ok(())
    }
    fn flush_page(&mut self, closed: bool) -> Result<(), String> {
        self.file
            .write_all(self.page.finish(0, self.index, self.nonce, closed))
            .map_err(|e| e.to_string())?;
        self.index += 1;
        self.page.clear();
        self.page_used = 0;
        Ok(())
    }
    pub(crate) fn sync(&mut self) -> Result<(), String> {
        if !self.page.is_empty() {
            self.flush_page(false)?;
        }
        self.file.flush().map_err(|e| e.to_string())?;
        self.file.get_ref().sync_data().map_err(|e| e.to_string())
    }
    pub(crate) fn close(&mut self, at_ns: u64, dropped: u32) -> Result<(), String> {
        if self.closed {
            return Ok(());
        }
        self.append(22, at_ns, &dropped.to_le_bytes())?;
        self.flush_page(true)?;
        self.closed = true;
        self.file.flush().map_err(|e| e.to_string())?;
        self.file.get_ref().sync_all().map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn synced_capacity_boundary_retains_space_to_close_and_reload() {
        let path = std::env::temp_dir().join(format!(
            "aevia-evidence-cap-{}-{}.aev",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = Writer::create(&path, 1, 3 * 512).unwrap();
        writer.append(99, 1, &[17; 256]).unwrap();
        writer.append(99, 2, &[18; 256]).unwrap();
        writer.sync().unwrap();
        assert!(writer.append(99, 3, &[19; 256]).is_err());
        writer.close(4, 0).unwrap();
        drop(writer);
        assert_eq!(std::fs::metadata(&path).unwrap().len(), 3 * 512);
        let session = crate::legacy::import(&path).unwrap();
        let health = session
            .raw_records
            .iter()
            .find(|r| r["kind"] == "legacy_capture")
            .unwrap();
        assert_eq!(health["closed"], true);
        assert_eq!(health["records"], 3);
        std::fs::remove_file(path).unwrap();
    }
}
