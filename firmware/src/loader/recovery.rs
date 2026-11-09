use super::update::{CHUNK, Error, Flash, Updater};

pub const LINE_SIZE: usize = 544;


pub enum Reply {
    Text(&'static [u8]),
    Reboot,
}

pub fn command<F: Flash>(updater: &mut Updater<F>, line: &[u8]) -> Result<Reply, Error> {
    let result = execute(updater, line);
    if result.is_err() {
        updater.abort();
    }
    result
}

fn execute<F: Flash>(updater: &mut Updater<F>, line: &[u8]) -> Result<Reply, Error> {
    let text = core::str::from_utf8(line).map_err(|_| Error::Command)?;
    let mut words = text.split_ascii_whitespace();
    let name = words.next().ok_or(Error::Command)?;
    let first = words.next();
    let second = words.next();
    if words.next().is_some() {
        return Err(Error::Command);
    }
    match (name, first, second) {
        ("INFO", None, None) => return Ok(Reply::Text(b"LOADER 1 MAX 4194304 CHUNK 256\n")),
        ("ABORT", None, None) => updater.abort(),
        ("RUN", None, None) => {
            updater.request_app()?;
            return Ok(Reply::Reboot);
        }
        ("BEGIN", Some(size), Some(hash)) => {
            let mut expected = [0; 32];
            if decode_hex(hash, &mut expected)? != 32 {
                return Err(Error::Command);
            }
            updater.begin(number(size)?, expected)?;
        }
        ("DATA", Some(offset), Some(hex)) => {
            let mut bytes = [0; CHUNK];
            let len = decode_hex(hex, &mut bytes)?;
            updater.data(number(offset)?, &bytes[..len])?;
        }
        ("COMMIT", None, None) => {
            updater.commit()?;
            return Ok(Reply::Reboot);
        }
        _ => return Err(Error::Command),
    }
    Ok(Reply::Text(b"OK\n"))
}

fn number(word: &str) -> Result<u32, Error> {
    if !word.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Error::Command);
    }
    word.parse().map_err(|_| Error::Command)
}

fn decode_hex(text: &str, output: &mut [u8]) -> Result<usize, Error> {
    if !text.len().is_multiple_of(2) || text.len() / 2 > output.len() {
        return Err(Error::Command);
    }
    fn digit(byte: u8) -> Result<u8, Error> {
        match byte {
            b'0'..=b'9' => Ok(byte - b'0'),
            b'a'..=b'f' => Ok(byte - b'a' + 10),
            b'A'..=b'F' => Ok(byte - b'A' + 10),
            _ => Err(Error::Command),
        }
    }
    for (pair, out) in text.as_bytes().as_chunks::<2>().0.iter().zip(output) {
        *out = digit(pair[0])? << 4 | digit(pair[1])?;
    }
    Ok(text.len() / 2)
}
