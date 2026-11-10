//! UART evidence is separate from parser/receiver policy failures. Successful
//! reads never erase the most recent hardware error or another port's counts.
use super::super::interface::{GnssTransportStatus, GnssUartError};

#[derive(Clone, Copy)]
enum RxKind {
    FifoOverflow,
    Glitch,
    Framing,
    Parity,
    Other,
}

impl GnssTransportStatus {
    fn record_rx(&mut self, port: u8, kind: RxKind, at_us: u64) {
        let Some(value) = port
            .checked_sub(1)
            .and_then(|index| self.ports.get_mut(index as usize))
        else {
            return;
        };
        value.rx_errors = value.rx_errors.saturating_add(1);
        let (counter, code) = match kind {
            RxKind::FifoOverflow => (&mut value.fifo_overflows, 1),
            RxKind::Glitch => (&mut value.glitches, 2),
            RxKind::Framing => (&mut value.framing_errors, 3),
            RxKind::Parity => (&mut value.parity_errors, 4),
            RxKind::Other => (&mut value.other_errors, 5),
        };
        *counter = counter.saturating_add(1);
        self.last_rx_error = Some(GnssUartError {
            port,
            code,
            received_us: at_us,
        });
    }

    fn record_bytes(&mut self, port: u8, count: usize) {
        if let Some(value) = port
            .checked_sub(1)
            .and_then(|index| self.ports.get_mut(index as usize))
        {
            value.rx_bytes = value.rx_bytes.saturating_add(count as u64);
        }
    }
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::*;
    use crate::device::interface::GnssUartPortStatus;
    use core::cell::Cell;
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
    use esp_hal::uart::RxError;

    const EMPTY_PORT: GnssUartPortStatus = GnssUartPortStatus {
        rx_bytes: 0,
        rx_errors: 0,
        fifo_overflows: 0,
        glitches: 0,
        framing_errors: 0,
        parity_errors: 0,
        other_errors: 0,
    };
    static STATUS: Mutex<CriticalSectionRawMutex, Cell<GnssTransportStatus>> =
        Mutex::new(Cell::new(GnssTransportStatus {
            ports: [EMPTY_PORT; 2],
            last_rx_error: None,
        }));

    pub(crate) fn record_rx(port: u8, error: RxError, at_us: u64) {
        let kind = match error {
            RxError::FifoOverflowed => RxKind::FifoOverflow,
            RxError::GlitchOccurred => RxKind::Glitch,
            RxError::FrameFormatViolated => RxKind::Framing,
            RxError::ParityMismatch => RxKind::Parity,
            _ => RxKind::Other,
        };
        STATUS.lock(|cell| {
            let mut value = cell.get();
            value.record_rx(port, kind, at_us);
            cell.set(value);
        });
    }

    pub(crate) fn record_bytes(port: u8, count: usize) {
        STATUS.lock(|cell| {
            let mut value = cell.get();
            value.record_bytes(port, count);
            cell.set(value);
        });
    }

    pub(crate) fn status() -> GnssTransportStatus {
        STATUS.lock(Cell::get)
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) use hardware::{record_bytes, record_rx, status};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn categories_and_ports_are_independent_and_success_retains_error_evidence() {
        let mut value = GnssTransportStatus::default();
        for (index, kind) in [
            RxKind::FifoOverflow,
            RxKind::Glitch,
            RxKind::Framing,
            RxKind::Parity,
            RxKind::Other,
        ]
        .into_iter()
        .enumerate()
        {
            value.record_rx(1, kind, index as u64);
        }
        value.record_rx(2, RxKind::FifoOverflow, 42);
        value.record_bytes(1, 128);
        value.record_bytes(2, 512);
        assert_eq!(value.ports[0].rx_errors, 5);
        assert_eq!(value.ports[0].fifo_overflows, 1);
        assert_eq!(value.ports[0].glitches, 1);
        assert_eq!(value.ports[0].framing_errors, 1);
        assert_eq!(value.ports[0].parity_errors, 1);
        assert_eq!(value.ports[0].other_errors, 1);
        assert_eq!(value.ports[0].rx_bytes, 128);
        assert_eq!(value.ports[1].rx_errors, 1);
        assert_eq!(value.ports[1].fifo_overflows, 1);
        assert_eq!(value.ports[1].glitches, 0);
        assert_eq!(value.ports[1].rx_bytes, 512);
        assert_eq!(
            value.last_rx_error,
            Some(GnssUartError {
                port: 2,
                code: 1,
                received_us: 42
            })
        );
        let previous = value;
        for port in [0, 3, u8::MAX] {
            value.record_rx(port, RxKind::Other, 100);
            value.record_bytes(port, 100);
        }
        assert_eq!(value, previous);
    }

    #[test]
    fn counters_saturate_instead_of_concealing_failures_after_wrap() {
        let mut value = GnssTransportStatus::default();
        value.ports[1].rx_errors = u32::MAX;
        value.ports[1].fifo_overflows = u32::MAX;
        value.ports[1].rx_bytes = u64::MAX;
        value.record_rx(2, RxKind::FifoOverflow, u64::MAX);
        value.record_bytes(2, usize::MAX);
        assert_eq!(value.ports[1].rx_errors, u32::MAX);
        assert_eq!(value.ports[1].fifo_overflows, u32::MAX);
        assert_eq!(value.ports[1].rx_bytes, u64::MAX);
        assert_eq!(value.last_rx_error.unwrap().received_us, u64::MAX);
    }
}
