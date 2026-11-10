//! UM980 serial interface, N4 binary messages and checksummed ASCII sentences.
//!
//! Hardware authority: `data/datasheets/UM980_User Manual_EN_R1.9.pdf`,
//! sections 1.2, 2.1 and 3.3: three UARTs, RESET_N low for at least 5 ms,
//! and more than 500 ms between supply removal and reapplication. The same
//! manual calls I2C/SPI/CAN reserved and unsupported; this driver does not
//! invent transports for those pins.
//!
//! The local manual contains no command or binary message reference. N4 framing,
//! BESTNAVXYZ, PPSSTATUS and the typed commands below preserve the mappings in
//! the existing `device/gnss.rs` and `device/timing/clock.rs` (which cite Unicore
//! N4 R1.14). Unknown binary messages and ASCII sentences remain available raw.
//! Receiver firmware can reject commands or output periods: a successful write
//! means bytes were sent, not that the receiver accepted the configuration.
//!
//! This module performs no probing, autobaud, persistent saves, mode changes,
//! estimator processing or background tasks. Feed UART bytes to the decoders;
//! reset a partial decoder after a caller-defined idle timeout. Timestamp PPS
//! edges separately in hardware. UART reads/writes inherit the transport's
//! blocking and timeout behavior. Use separate `Um980` values for split COM1
//! command and COM2 data paths when needed.

mod commands;
mod framing;
mod messages;

pub use commands::{Baud, Command, CommandError, LogMessage, Period, Port};
pub use framing::{
    AsciiDecoder, AsciiSentence, BinaryDecoder, BinaryFrame, DecodeError, Decoder, Frame,
    SentenceKind, crc32,
};
pub use messages::{BestNavXyz, Header, PpsStatus};

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use embedded_io::{Read, Write};

/// UART operation errors, preserving the original transport error.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Io(E),
    Command(CommandError),
    /// A transport returned zero progress for a nonempty write.
    WriteZero,
    /// A transport violated the `embedded-io` byte-count contract.
    InvalidByteCount,
}

/// One UM980 UART endpoint. Constructing this value never touches the receiver.
pub struct Um980<IO> {
    io: IO,
}

impl<IO> Um980<IO> {
    pub const fn new(io: IO) -> Self {
        Self { io }
    }

    pub fn release(self) -> IO {
        self.io
    }

    /// Access the underlying transport, e.g. to apply the host UART's baud rate
    /// after the receiver's `SetBaud` command has been transmitted and flushed.
    pub fn io_mut(&mut self) -> &mut IO {
        &mut self.io
    }
}

impl<IO: Write> Um980<IO> {
    /// Validate and encode the entire command before writing any bytes. Flushes
    /// transmission, but does not wait for or interpret a receiver response.
    /// Raw commands must be one printable ASCII line without CR/LF or NUL.
    pub fn command(&mut self, command: Command<'_>) -> Result<(), Error<IO::Error>> {
        let mut buffer = [0; 128];
        let len = command.encode(&mut buffer).map_err(Error::Command)?;
        self.write_all_checked(&buffer[..len])?;
        self.io.flush().map_err(Error::Io)
    }

    /// Send already encoded correction data (for example RTCM v3). These bytes
    /// are opaque: framing, checksums and correction age remain the caller's
    /// responsibility. No CR/LF or other bytes are added.
    pub fn corrections(&mut self, bytes: &[u8]) -> Result<(), Error<IO::Error>> {
        self.write_all_checked(bytes)?;
        self.io.flush().map_err(Error::Io)
    }

    fn write_all_checked(&mut self, mut bytes: &[u8]) -> Result<(), Error<IO::Error>> {
        while !bytes.is_empty() {
            let n = self.io.write(bytes).map_err(Error::Io)?;
            if n == 0 {
                return Err(Error::WriteZero);
            }
            if n > bytes.len() {
                return Err(Error::InvalidByteCount);
            }
            bytes = &bytes[n..];
        }
        Ok(())
    }
}

impl<IO: Read> Um980<IO> {
    /// Read bytes without discarding any receiver responses. `Ok(0)` retains
    /// the underlying transport's EOF meaning; it does not indicate a fix.
    pub fn read(&mut self, bytes: &mut [u8]) -> Result<usize, Error<IO::Error>> {
        let n = self.io.read(bytes).map_err(Error::Io)?;
        if n > bytes.len() {
            return Err(Error::InvalidByteCount);
        }
        Ok(n)
    }
}

/// Assert RESET_N for the datasheet minimum 5 ms and release it. Configure the
/// pin as open drain where required by the board. Boot readiness is not implied
/// by release; determine readiness from receiver output or a version response.
pub fn hardware_reset<P: OutputPin, D: DelayNs>(
    reset_n: &mut P,
    delay: &mut D,
) -> Result<(), P::Error> {
    reset_n.set_low()?;
    delay.delay_ms(5);
    reset_n.set_high()
}

#[cfg(test)]
mod tests;
