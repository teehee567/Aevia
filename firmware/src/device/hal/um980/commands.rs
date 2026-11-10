use core::fmt::{self, Write};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Com1,
    Com2,
    Com3,
}

impl Port {
    const fn number(self) -> u8 {
        match self {
            Self::Com1 => 1,
            Self::Com2 => 2,
            Self::Com3 => 3,
        }
    }
}

/// Baud rates used by the existing firmware's receiver probe. Other rates can
/// be requested with `Command::Raw` after checking the fitted firmware manual.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Baud {
    B115200,
    B460800,
    B921600,
}

impl Baud {
    pub const fn bits_per_second(self) -> u32 {
        match self {
            Self::B115200 => 115_200,
            Self::B460800 => 460_800,
            Self::B921600 => 921_600,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogMessage {
    BestNavXyzBinary,
    PpsStatusBinary,
}

impl LogMessage {
    const fn name(self) -> &'static str {
        match self {
            Self::BestNavXyzBinary => "BESTNAVXYZB",
            Self::PpsStatusBinary => "PPSSTATUSB",
        }
    }
}

/// Whole-millisecond output interval; requests are validated syntactically.
/// Receiver firmware/mode determines which actual periods it accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Period(u32);

impl Period {
    pub const fn from_millis(millis: u32) -> Result<Self, CommandError> {
        if millis == 0 || millis > 86_400_000 {
            return Err(CommandError::PeriodOutOfRange);
        }
        Ok(Self(millis))
    }

    pub const fn as_millis(self) -> u32 {
        self.0
    }
}

/// Commands with existing in-repository wire evidence. All commands are
/// volatile unless a caller explicitly sends a persistence command using Raw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command<'a> {
    Version {
        port: Port,
    },
    QueryConfig,
    SetBaud {
        port: Port,
        baud: Baud,
    },
    Unlog {
        port: Port,
    },
    Log {
        port: Port,
        message: LogMessage,
        period: Period,
    },
    /// Existing board timing profile: GPST rising edge, 100 ms pulse, 1 s
    /// period, zero user/RF delay. Read back CONFIG before using timing data.
    PpsGpsRisingOneHz,
    /// Complete one-line command body; CR/LF is appended by `encode`.
    Raw(&'a str),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError {
    InvalidCharacters,
    Empty,
    TooLong,
    BufferTooSmall,
    PeriodOutOfRange,
}

impl Command<'_> {
    /// Encode the complete CRLF-terminated wire command. A too-small output
    /// buffer is left unchanged. Maximum command body length is 126 bytes.
    pub fn encode(self, output: &mut [u8]) -> Result<usize, CommandError> {
        let mut command = Text {
            bytes: [0; 128],
            len: 0,
        };
        let encoded = match self {
            Self::Version { port } => write!(command, "VERSIONB COM{}", port.number()),
            Self::QueryConfig => command.write_str("CONFIG"),
            Self::SetBaud { port, baud } => write!(
                command,
                "CONFIG COM{} {}",
                port.number(),
                baud.bits_per_second()
            ),
            Self::Unlog { port } => write!(command, "UNLOG COM{}", port.number()),
            Self::Log {
                port,
                message,
                period,
            } => write!(
                command,
                "{} COM{} {}.{:03}",
                message.name(),
                port.number(),
                period.0 / 1000,
                period.0 % 1000
            ),
            Self::PpsGpsRisingOneHz => {
                command.write_str("CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0")
            }
            Self::Raw(body) => {
                if body.is_empty() {
                    return Err(CommandError::Empty);
                }
                if !body.bytes().all(|byte| (0x20..=0x7e).contains(&byte)) {
                    return Err(CommandError::InvalidCharacters);
                }
                if body.len() > 126 {
                    return Err(CommandError::TooLong);
                }
                command.write_str(body)
            }
        };
        encoded.map_err(|_| CommandError::TooLong)?;
        command
            .write_str("\r\n")
            .map_err(|_| CommandError::TooLong)?;
        if output.len() < command.len {
            return Err(CommandError::BufferTooSmall);
        }
        output[..command.len].copy_from_slice(&command.bytes[..command.len]);
        Ok(command.len)
    }
}

struct Text {
    bytes: [u8; 128],
    len: usize,
}

impl Write for Text {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if text.len() > self.bytes.len() - self.len {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}
