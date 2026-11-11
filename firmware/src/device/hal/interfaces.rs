//! Common boundaries for MCU peripherals, storage and communication stacks.
//!
//! I2C, SPI, GPIO and delay use the re-exported `embedded_hal` traits. UART
//! and USB byte streams use `embedded_io::{Read, Write}`. Implementations must
//! preserve errors and put finite deadlines on hardware operations. Radio
//! interfaces describe services supplied by a vendor stack, not radio registers.

/// Calibrated ADC voltage at a physical input (not the divided source voltage).
pub trait AnalogInput {
    type Error;
    fn read_microvolts(&mut self) -> Result<u32, Self::Error>;
}

/// Temperature of the explicitly identified sensor, in degrees Celsius.
pub trait TemperatureSensor {
    type Error;
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error>;
}

/// A monotonic counter in nanoseconds; it need not share the GNSS epoch.
pub trait MonotonicClock {
    fn now_ns(&self) -> u64;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CapturedEdge {
    pub time_ns: u64,
    /// Number of edges lost before this capture, if tracked by the backend.
    pub missed: u32,
}

/// Hardware-latched PPS / IMU DRDY edges. Never substitute a task wakeup time.
pub trait EdgeCapture {
    type Error;
    fn take_edge(&mut self) -> Result<Option<CapturedEdge>, Self::Error>;
}

/// One logical sector is always 512 bytes, independent of bus block size.
pub trait BlockStorage {
    type Error;
    fn sector_count(&self) -> u64;
    fn read_sector(&mut self, sector: u64, data: &mut [u8; 512]) -> Result<(), Self::Error>;
    fn write_sector(&mut self, sector: u64, data: &[u8; 512]) -> Result<(), Self::Error>;
    fn flush(&mut self) -> Result<(), Self::Error>;
}

/// Optional bounded contiguous reads, without changing single-sector writes.
/// Accept one to eight complete 512-byte sectors. Validate the entire range
/// before I/O; propagate transfer/CRC/status errors without partial success.
/// On error the caller must discard all returned buffer contents.
pub trait ReadSectors: BlockStorage {
    fn read_sectors(&mut self, first_sector: u64, data: &mut [u8]) -> Result<(), Self::Error>;
}

/// Addressed, byte-oriented NOR operations; implementations enforce alignment,
/// erase geometry and flash execution/multicore exclusion.
pub trait NorStorage {
    type Error;
    fn capacity_bytes(&self) -> u64;
    fn erase_size_bytes(&self) -> u32;
    fn read(&mut self, offset: u64, data: &mut [u8]) -> Result<(), Self::Error>;
    fn program(&mut self, offset: u64, data: &[u8]) -> Result<(), Self::Error>;
    fn erase(&mut self, offset: u64, length: u64) -> Result<(), Self::Error>;
}

/// Safe access to PSRAM owned exclusively by the backend. No raw pointer API.
pub trait RamStorage {
    type Error;
    fn capacity_bytes(&self) -> usize;
    fn read(&mut self, offset: usize, data: &mut [u8]) -> Result<(), Self::Error>;
    fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), Self::Error>;
}

/// A frame-oriented interface for a vendor Wi-Fi / 802.15.4 MAC backend.
/// The backend's contract defines MTU and whether MAC headers are included.
/// Oversized receive frames must return an error instead of silent truncation.
pub trait PacketTransport {
    type Error;
    fn mtu(&self) -> usize;
    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
    /// `None` means no packet available; the call must not wait indefinitely.
    fn try_receive(&mut self, buffer: &mut [u8]) -> Result<Option<usize>, Self::Error>;
}

/// Bluetooth ATT/GATT service supplied by a vendor stack. Link establishment,
/// authentication, advertising and characteristic registration belong there.
pub trait GattServer {
    type Error;
    fn connected(&self) -> bool;
    fn notify(&mut self, characteristic: u16, value: &[u8]) -> Result<(), Self::Error>;
    /// Returns (characteristic handle, bytes written); no silent truncation.
    fn try_receive_write(&mut self, buffer: &mut [u8])
    -> Result<Option<(u16, usize)>, Self::Error>;
}

/// USB connection state in addition to the standard embedded-io byte stream.
pub trait UsbConnection {
    fn configured(&self) -> bool;
    fn data_terminal_ready(&self) -> bool;
}

/// Raw CAN capability for an optional external transceiver/controller. No CAN
/// transceiver is fitted on V2 Mini; this is an integration boundary only.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CanId {
    Standard(u16),
    Extended(u32),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanFrame {
    id: CanId,
    data: [u8; 8],
    len: u8,
}

impl CanFrame {
    pub fn new(id: CanId, data: &[u8]) -> Option<Self> {
        if data.len() > 8
            || matches!(id, CanId::Standard(v) if v > 0x7ff)
            || matches!(id, CanId::Extended(v) if v > 0x1fff_ffff)
        {
            return None;
        }
        let mut frame = Self {
            id,
            data: [0; 8],
            len: data.len() as u8,
        };
        frame.data[..data.len()].copy_from_slice(data);
        Some(frame)
    }
    pub const fn id(&self) -> CanId {
        self.id
    }
    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

pub trait CanTransport {
    type Error;
    fn send(&mut self, frame: &CanFrame) -> Result<(), Self::Error>;
    fn try_receive(&mut self) -> Result<Option<CanFrame>, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn can_boundaries_reject_invalid_ids_and_lengths() {
        assert!(CanFrame::new(CanId::Standard(0x7ff), &[1; 8]).is_some());
        assert!(CanFrame::new(CanId::Standard(0x800), &[]).is_none());
        assert!(CanFrame::new(CanId::Extended(0x2000_0000), &[]).is_none());
        assert!(CanFrame::new(CanId::Standard(1), &[0; 9]).is_none());
    }
}
