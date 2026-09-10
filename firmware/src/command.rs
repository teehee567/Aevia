//! Bounded, line-framed USB commands. Packet boundaries have no protocol meaning.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Bootloader,
    Stream(u32),
    Stop,
    LcdReinitialize,
    LcdLight(u8),
    LcdTest,
    GnssBridge,
    GnssNormal,
    Invalid,
}

#[derive(Default)]
pub struct Decoder {
    line: heapless::Vec<u8, 32>,
    discarding: bool,
}

impl Decoder {
    /// An oversized or non-ASCII line is discarded through its terminator.
    /// Call `reset` on disconnect so two connections cannot form one command.
    pub fn push(&mut self, byte: u8) -> Option<Command> {
        if byte == b'\n' {
            let result = if self.discarding {
                Some(Command::Invalid)
            } else if self.line.is_empty() {
                None
            } else {
                Some(parse(self.line.strip_suffix(b"\r").unwrap_or(&self.line)))
            };
            self.reset();
            return result;
        }
        if !self.discarding
            && (!byte.is_ascii_graphic() && byte != b' ' && byte != b'\r'
                || self.line.push(byte).is_err())
        {
            self.discarding = true;
        }
        None
    }

    pub fn reset(&mut self) {
        self.line.clear();
        self.discarding = false;
    }
}

fn parse(line: &[u8]) -> Command {
    match line {
        b"BOOTLOADER" => Command::Bootloader,
        b"STOP" => Command::Stop,
        b"LCDREINIT" => Command::LcdReinitialize,
        b"LCDTEST" => Command::LcdTest,
        b"GNSSBRIDGE" => Command::GnssBridge,
        b"GNSSNORMAL" => Command::GnssNormal,
        b"LCDLIGHT 0" => Command::LcdLight(0),
        b"LCDLIGHT 1" => Command::LcdLight(1),
        b"LCDLIGHT 5" => Command::LcdLight(5),
        b"LCDLIGHT 10" => Command::LcdLight(10),
        _ => {
            let Some(token) = line.strip_prefix(b"STREAM ") else {
                return Command::Invalid;
            };
            if token.len() != 8 || !token.iter().all(u8::is_ascii_hexdigit) {
                return Command::Invalid;
            }
            let text = core::str::from_utf8(token).unwrap(); // ASCII checked above.
            Command::Stream(u32::from_str_radix(text, 16).unwrap())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn feed(decoder: &mut Decoder, bytes: &[u8]) -> Vec<Command> {
        bytes
            .iter()
            .filter_map(|byte| decoder.push(*byte))
            .collect()
    }

    #[test]
    fn reboot_requires_a_complete_exact_command() {
        let mut decoder = Decoder::default();
        assert!(feed(&mut decoder, b"BOOTLOAD").is_empty());
        assert_eq!(feed(&mut decoder, b"ER\r\n"), [Command::Bootloader]);
        assert_eq!(
            feed(&mut decoder, b"XBOOTLOADER\nBOOTLOADERX\n"),
            [Command::Invalid; 2]
        );
    }

    #[test]
    fn overflow_cannot_turn_a_suffix_into_a_reboot() {
        let mut decoder = Decoder::default();
        feed(&mut decoder, &[b'x'; 64]);
        assert_eq!(
            feed(&mut decoder, b"BOOTLOADER\nSTOP\n"),
            [Command::Invalid, Command::Stop]
        );
        assert_eq!(feed(&mut decoder, b"\0BOOTLOADER\n"), [Command::Invalid]);
        assert_eq!(feed(&mut decoder, b"BOOT\rLOADER\n"), [Command::Invalid]);
    }

    #[test]
    fn disconnect_drops_partial_commands() {
        let mut decoder = Decoder::default();
        feed(&mut decoder, b"BOOT");
        decoder.reset();
        assert_eq!(feed(&mut decoder, b"LOADER\n"), [Command::Invalid]);
    }

    #[test]
    fn stream_nonce_is_exactly_eight_hex_digits() {
        let mut decoder = Decoder::default();
        assert_eq!(
            feed(&mut decoder, b"STREAM 0123abCD\nSTREAM 00000000\n"),
            [Command::Stream(0x0123abcd), Command::Stream(0)]
        );
        assert_eq!(
            feed(
                &mut decoder,
                b"STREAM +1234567\nSTREAM 1234567\nSTREAM 123456789\n"
            ),
            [Command::Invalid; 3]
        );
    }

    #[test]
    fn every_packet_split_has_the_same_result() {
        let bytes = b"STREAM 0123abcd\r\nBOOTLOADER\n";
        for split in 0..=bytes.len() {
            let mut decoder = Decoder::default();
            let mut commands = feed(&mut decoder, &bytes[..split]);
            commands.extend(feed(&mut decoder, &bytes[split..]));
            assert_eq!(commands, [Command::Stream(0x0123abcd), Command::Bootloader]);
        }
    }

    #[test]
    fn bridge_commands_are_exact_and_packet_independent() {
        let bytes = b"GNSSBRIDGE\r\nGNSSNORMAL\n";
        for split in 0..=bytes.len() {
            let mut decoder = Decoder::default();
            let mut commands = feed(&mut decoder, &bytes[..split]);
            commands.extend(feed(&mut decoder, &bytes[split..]));
            assert_eq!(commands, [Command::GnssBridge, Command::GnssNormal]);
        }
        assert_eq!(
            feed(&mut Decoder::default(), b"XGNSSBRIDGE\nGNSSNORMALX\n"),
            [Command::Invalid; 2]
        );
    }
}
