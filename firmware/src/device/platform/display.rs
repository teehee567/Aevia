//! Bounded FIFO writes for the write-only panel. The vendor SpiBus waits
//! indefinitely for UPDATE/USR, including inside an async future's poll.
//! FIFO setup follows pinned esp-hal 1.2.2 spi/master/low_level/{mod,v3}.rs.
//! Native reset restores that SDK configuration without waiting for USR to end.
use aevia_device_hal::embedded_hal::spi;

const FIFO_BYTES: usize = 64;
const TRANSFER_US: u64 = 2_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error {
    Timeout,
    Unsupported,
}
impl spi::Error for Error {
    fn kind(&self) -> spi::ErrorKind {
        spi::ErrorKind::Other
    }
}

// Internal seam: native registers and a fault-injecting register model execute
// exactly the same transfer/deadline/recovery procedure.
pub(crate) trait Registers {
    fn now_us(&self) -> u64;
    fn busy(&self) -> bool;
    fn updating(&self) -> bool;
    fn load(&mut self, bytes: &[u8]);
    fn update(&mut self);
    fn start(&mut self);
    fn reset(&mut self);
}

pub(crate) struct BoundedBus<R>(R);
impl<R: Registers> BoundedBus<R> {
    fn pending(&self, updating: bool) -> bool {
        if updating {
            self.0.updating()
        } else {
            self.0.busy()
        }
    }

    fn wait(&mut self, deadline: u64, updating: bool) -> Result<(), Error> {
        while self.pending(updating) {
            if self.0.now_us() >= deadline {
                // Completion can occur while a higher-priority task preempts
                // the clock read. Recheck hardware before treating it as hung.
                if !self.pending(updating) {
                    return Ok(());
                }
                // Reset without waiting for the transfer that just timed out.
                self.0.reset();
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }
}
impl<R: Registers> spi::ErrorType for BoundedBus<R> {
    type Error = Error;
}
impl<R: Registers> spi::SpiBus for BoundedBus<R> {
    fn read(&mut self, _: &mut [u8]) -> Result<(), Error> {
        Err(Error::Unsupported)
    }
    fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Error> {
        Err(Error::Unsupported)
    }
    fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Error> {
        Err(Error::Unsupported)
    }
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for chunk in bytes.chunks(FIFO_BYTES) {
            let deadline = self.0.now_us().saturating_add(TRANSFER_US);
            self.wait(deadline, false)?;
            self.0.load(chunk);
            self.0.update();
            self.wait(deadline, true)?;
            self.0.start();
            self.wait(deadline, false)?;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Error> {
        self.wait(self.0.now_us().saturating_add(TRANSFER_US), false)
    }
}

#[cfg(target_arch = "riscv32")]
pub(super) mod native {
    use super::*;
    use esp_hal::{
        Blocking,
        peripherals::{GPIO16, GPIO17, SPI3},
        spi::master::{ConfigError, Spi},
    };

    pub(crate) type DisplayBus = BoundedBus<PanelRegisters>;
    pub(crate) struct PanelRegisters {
        // Retain the SDK peripheral/pin guards. Never call its unbounded I/O.
        _spi: Spi<'static, Blocking>,
        configuration: [u32; 11],
    }

    fn registers() -> &'static esp32s31::spi2::RegisterBlock {
        // SAFETY: PanelRegisters owns SPI3 exclusively, with no DMA/interrupts.
        unsafe { &*esp32s31::SPI3::ptr() }
    }

    impl DisplayBus {
        pub(crate) fn new(
            peripheral: SPI3<'static>,
            sclk: GPIO16<'static>,
            mosi: GPIO17<'static>,
        ) -> Result<Self, ConfigError> {
            // Consume SPI3 itself: accepting a type-erased Spi would also
            // allow a SPI2 handle, which cannot authorize these PAC accesses.
            let spi = aevia_device_hal::esp32s31::init_display_spi(peripheral, sclk, mosi)?;
            // The SDK chose the function clock source/dividers during setup.
            // Keep those clocks enabled for this exclusively owned adapter.
            let clocks = unsafe { &*esp32s31::HP_SYS_CLKRST::ptr() };
            clocks
                .gpspi3_ctrl0()
                .modify(|_, w| w.hs_clk_en().set_bit().mst_clk_en().set_bit());
            let r = registers();
            r.user().modify(|_, w| {
                w.usr_miso()
                    .set_bit()
                    .usr_mosi()
                    .set_bit()
                    .doutdin()
                    .set_bit()
                    .usr_dummy()
                    .clear_bit()
                    .sio()
                    .clear_bit()
            });
            let configuration = [
                r.clock().read().bits(),
                r.ctrl().read().bits(),
                r.user().read().bits(),
                r.user1().read().bits(),
                r.user2().read().bits(),
                r.misc().read().bits(),
                r.din_mode().read().bits(),
                r.din_num().read().bits(),
                r.dout_mode().read().bits(),
                r.slave().read().bits(),
                r.clk_gate().read().bits(),
            ];
            Ok(Self(PanelRegisters {
                _spi: spi,
                configuration,
            }))
        }
    }

    impl Registers for PanelRegisters {
        fn now_us(&self) -> u64 {
            esp_hal::time::Instant::now()
                .duration_since_epoch()
                .as_micros()
        }
        fn busy(&self) -> bool {
            registers().cmd().read().usr().bit_is_set()
        }
        fn updating(&self) -> bool {
            registers().cmd().read().update().bit_is_set()
        }
        fn load(&mut self, bytes: &[u8]) {
            let r = registers();
            r.ms_dlen()
                .write(|w| unsafe { w.ms_data_bitlen().bits((bytes.len() * 8 - 1) as u32) });
            for (i, chunk) in bytes.chunks(4).enumerate() {
                let mut word = [0; 4];
                word[..chunk.len()].copy_from_slice(chunk);
                r.w(i)
                    .write(|w| unsafe { w.bits(u32::from_le_bytes(word)) });
            }
        }
        fn update(&mut self) {
            registers().cmd().modify(|_, w| w.update().set_bit());
        }
        fn start(&mut self) {
            registers()
                .dma_int_clr()
                .write(|w| w.trans_done().clear_bit_by_one());
            registers().cmd().modify(|_, w| w.usr().set_bit());
        }
        fn reset(&mut self) {
            let clocks = unsafe { &*esp32s31::HP_SYS_CLKRST::ptr() };
            clocks.gpspi3_ctrl0().modify(|_, w| w.rst_en().set_bit());
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            clocks.gpspi3_ctrl0().modify(|_, w| w.rst_en().clear_bit());
            let r = registers();
            // Restore only the SDK's non-DMA mode/clock configuration. GPIO
            // matrix ownership survives this peripheral reset. The panel
            // driver sees an error, blanks the backlight and reinitializes.
            unsafe {
                // Restore register/module clocks before configuration writes,
                // matching the SDK's enable_peripheral_clock/init ordering.
                r.clk_gate().write(|w| w.bits(self.configuration[10]));
                r.clock().write(|w| w.bits(self.configuration[0]));
                r.ctrl().write(|w| w.bits(self.configuration[1]));
                r.user().write(|w| w.bits(self.configuration[2]));
                r.user1().write(|w| w.bits(self.configuration[3]));
                r.user2().write(|w| w.bits(self.configuration[4]));
                r.misc().write(|w| w.bits(self.configuration[5]));
                r.din_mode().write(|w| w.bits(self.configuration[6]));
                r.din_num().write(|w| w.bits(self.configuration[7]));
                r.dout_mode().write(|w| w.bits(self.configuration[8]));
                r.slave().write(|w| w.bits(self.configuration[9]));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use spi::SpiBus;
    #[derive(Default)]
    struct Model {
        time: Cell<u64>,
        stuck_update: bool,
        stuck_transfer: bool,
        busy: bool,
        resets: usize,
        starts: usize,
        loaded: usize,
    }
    impl Registers for Model {
        fn now_us(&self) -> u64 {
            let now = self.time.get();
            self.time.set(now + 10);
            now
        }
        fn busy(&self) -> bool {
            self.busy
        }
        fn updating(&self) -> bool {
            self.stuck_update
        }
        fn load(&mut self, bytes: &[u8]) {
            assert!(bytes.len() <= 64);
            self.loaded += bytes.len();
        }
        fn update(&mut self) {}
        fn start(&mut self) {
            self.starts += 1;
            self.busy = self.stuck_transfer;
        }
        fn reset(&mut self) {
            self.resets += 1;
            self.busy = false;
            self.stuck_update = false;
            self.stuck_transfer = false;
        }
    }
    #[test]
    fn stuck_update_and_transfer_return_errors_and_the_next_attempt_recovers() {
        for stuck_update in [true, false] {
            let mut bus = BoundedBus(Model {
                stuck_update,
                stuck_transfer: !stuck_update,
                ..Default::default()
            });
            assert_eq!(bus.write(&[1; 64]), Err(Error::Timeout));
            assert_eq!(bus.0.resets, 1);
            assert!(bus.0.time.get() <= TRANSFER_US + 20);
            bus.flush().unwrap();
            bus.write(&[2; 480]).unwrap();
            assert_eq!(bus.0.loaded, 64 + 480);
        }
    }
    #[test]
    fn previously_stuck_bus_is_reset_before_loading_more_data() {
        let mut bus = BoundedBus(Model {
            busy: true,
            ..Default::default()
        });
        assert_eq!(bus.write(&[1]), Err(Error::Timeout));
        assert_eq!(bus.0.loaded, 0);
        assert_eq!(bus.0.starts, 0);
        bus.write(&[1]).unwrap();
        assert_eq!(bus.0.starts, 1);
    }

    #[test]
    fn completed_update_or_transfer_survives_clock_read_preemption() {
        struct Race {
            update: bool,
            sampled_clock: Cell<bool>,
            updating: Cell<bool>,
            busy: Cell<bool>,
            resets: usize,
        }
        impl Registers for Race {
            fn now_us(&self) -> u64 {
                if self.sampled_clock.replace(true) {
                    self.updating.set(false);
                    self.busy.set(false);
                    TRANSFER_US + 1
                } else {
                    0
                }
            }
            fn busy(&self) -> bool {
                self.busy.get()
            }
            fn updating(&self) -> bool {
                self.updating.get()
            }
            fn load(&mut self, _: &[u8]) {}
            fn update(&mut self) {
                self.updating.set(self.update);
            }
            fn start(&mut self) {
                self.busy.set(!self.update);
            }
            fn reset(&mut self) {
                self.resets += 1;
            }
        }
        for update in [true, false] {
            let mut bus = BoundedBus(Race {
                update,
                sampled_clock: Cell::new(false),
                updating: Cell::new(false),
                busy: Cell::new(false),
                resets: 0,
            });
            bus.write(&[0; 6]).unwrap();
            assert_eq!(bus.0.resets, 0);
        }
    }

    #[test]
    fn panel_reinitializes_after_timeout_with_cs_released_and_backlight_off() {
        use aevia_device_hal::embedded_hal::{
            delay::DelayNs,
            digital::{ErrorType, OutputPin},
        };
        use aevia_device_hal::{
            bus::ExclusiveSpiDevice,
            display::{Config, St7789},
        };
        use core::convert::Infallible;
        struct Pin<'a>(&'a Cell<bool>);
        impl ErrorType for Pin<'_> {
            type Error = Infallible;
        }
        impl OutputPin for Pin<'_> {
            fn set_low(&mut self) -> Result<(), Infallible> {
                self.0.set(false);
                Ok(())
            }
            fn set_high(&mut self) -> Result<(), Infallible> {
                self.0.set(true);
                Ok(())
            }
        }
        struct Delay;
        impl DelayNs for Delay {
            fn delay_ns(&mut self, _: u32) {}
        }

        let (cs, dc, reset, backlight) = (
            Cell::new(true),
            Cell::new(false),
            Cell::new(true),
            Cell::new(false),
        );
        let bus = BoundedBus(Model {
            stuck_transfer: true,
            ..Default::default()
        });
        let mut spi = ExclusiveSpiDevice::new(bus, Pin(&cs), Delay);
        spi.initialize().unwrap();
        let mut panel = St7789::new(
            spi,
            Pin(&dc),
            Pin(&reset),
            Pin(&backlight),
            Delay,
            Config::v2_mini(),
        );
        assert!(panel.initialize().is_err());
        assert!(cs.get());
        assert!(!backlight.get());
        assert!(panel.write_row(0, 0, &[0; 240]).is_err());
        panel.initialize().unwrap();
        panel.write_row(0, 0, &[0; 240]).unwrap();
        panel.set_backlight(true).unwrap();
        assert!(cs.get());
        assert!(backlight.get());
    }
}
