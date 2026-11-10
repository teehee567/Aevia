//! Bounded full-duplex SPI2 transfers for SCH16T. A completed late burst is
//! rejected by acquisition; a wedged controller is reset here before returning.
use aevia_device_hal::embedded_hal::spi;

const FIFO_BYTES: usize = 64;
// A six-byte frame normally takes 6 us at 8 MHz, but interrupt execution may
// delay polling. Bound a wedged controller by the full read budget instead of
// resetting completed/usable transfers at an arbitrary smaller threshold.
// Acquisition independently rejects every complete burst older than 600 us.
const TRANSFER_US: u32 = 600;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error {
    Timeout,
}
impl spi::Error for Error {
    fn kind(&self) -> spi::ErrorKind {
        spi::ErrorKind::Other
    }
}

pub(crate) trait Registers {
    fn now_ticks(&self) -> u32;
    fn timeout_ticks(&self) -> u32 {
        TRANSFER_US
    }
    fn micros_for_ticks(&self, ticks: u32) -> u32 {
        ticks
    }
    fn busy(&self) -> bool;
    fn updating(&self) -> bool;
    fn load(&mut self, bytes: &[u8]);
    fn receive(&self, bytes: &mut [u8]);
    fn update(&mut self);
    fn start(&mut self);
    fn reset(&mut self);
    fn timeout(&mut self, _phase: u8, _elapsed_us: u32) {}
    fn completed(&mut self, _elapsed_us: u32) {}
}

pub(crate) struct BoundedBus<R>(R);
impl<R: Registers> BoundedBus<R> {
    fn pending(&self, phase: u8) -> bool {
        if phase == 2 {
            self.0.updating()
        } else {
            self.0.busy()
        }
    }

    fn wait(&mut self, started: u32, phase: u8) -> Result<(), Error> {
        while self.pending(phase) {
            let ticks = super::cycles::elapsed(started, self.0.now_ticks()).unwrap_or(u32::MAX);
            if ticks >= self.0.timeout_ticks() {
                // An interrupt can run between the busy read and clock read.
                // Completion during that interval must not reset a healthy
                // controller. Acquisition still rejects a burst older than its
                // independent 600 us measurement deadline.
                if !self.pending(phase) {
                    return Ok(());
                }
                self.0.timeout(phase, self.0.micros_for_ticks(ticks));
                self.0.reset();
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    fn exchange(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        for chunk in bytes.chunks_mut(FIFO_BYTES) {
            let started = self.0.now_ticks();
            self.wait(started, 1)?;
            self.0.load(chunk);
            self.0.update();
            self.wait(started, 2)?;
            self.0.start();
            self.wait(started, 3)?;
            self.0.receive(chunk);
            let ticks = super::cycles::elapsed(started, self.0.now_ticks()).unwrap_or(u32::MAX);
            self.0.completed(self.0.micros_for_ticks(ticks));
        }
        Ok(())
    }
}
impl<R: Registers> spi::ErrorType for BoundedBus<R> {
    type Error = Error;
}
impl<R: Registers> spi::SpiBus for BoundedBus<R> {
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        bytes.fill(0);
        self.exchange(bytes)
    }
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for chunk in bytes.chunks(FIFO_BYTES) {
            let mut wire = [0; FIFO_BYTES];
            wire[..chunk.len()].copy_from_slice(chunk);
            self.exchange(&mut wire[..chunk.len()])?;
        }
        Ok(())
    }
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let count = read.len().max(write.len());
        for offset in (0..count).step_by(FIFO_BYTES) {
            let n = (count - offset).min(FIFO_BYTES);
            let mut wire = [0; FIFO_BYTES];
            let sent = write.len().saturating_sub(offset).min(n);
            wire[..sent]
                .copy_from_slice(&write[offset.min(write.len())..offset.min(write.len()) + sent]);
            self.exchange(&mut wire[..n])?;
            let received = read.len().saturating_sub(offset).min(n);
            if received != 0 {
                read[offset..offset + received].copy_from_slice(&wire[..received]);
            }
        }
        Ok(())
    }
    fn transfer_in_place(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        self.exchange(bytes)
    }
    fn flush(&mut self) -> Result<(), Error> {
        self.wait(self.0.now_ticks(), 4)
    }
}

#[cfg(target_arch = "riscv32")]
pub(super) mod native {
    use super::super::cycles::native::{CycleClock, ImuDelay};
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
    use esp_hal::{
        Blocking,
        peripherals::{GPIO11, GPIO12, GPIO13, SPI2},
        spi::master::{ConfigError, Spi},
    };

    pub(crate) type ImuBus<'d> = BoundedBus<ImuRegisters<'d>>;
    pub(crate) struct ImuRegisters<'d> {
        _spi: Spi<'d, Blocking>,
        configuration: [u32; 11],
        clock: Option<CycleClock>,
        timeout_ticks: u32,
    }

    static CONTEXT: AtomicU8 = AtomicU8::new(0);
    static TIMEOUTS: AtomicU32 = AtomicU32::new(0);
    static PHASE: AtomicU8 = AtomicU8::new(0);
    static LAST_CONTEXT: AtomicU8 = AtomicU8::new(0);
    static LAST_ELAPSED: AtomicU32 = AtomicU32::new(0);
    static MAX_ELAPSED: AtomicU32 = AtomicU32::new(0);
    static LAST_CMD: AtomicU32 = AtomicU32::new(0);
    static LAST_CLOCK: AtomicU32 = AtomicU32::new(0);
    static LAST_SOURCE: AtomicU32 = AtomicU32::new(0);
    static CYCLE_ACTIVE: AtomicBool = AtomicBool::new(false);
    static CYCLE_HZ: AtomicU32 = AtomicU32::new(0);

    pub(crate) fn set_context(acquiring: bool) {
        CONTEXT.store(u8::from(acquiring), Ordering::Relaxed);
    }

    pub(crate) fn diagnostics(
        mut value: crate::device::interface::ImuDiagnostics,
    ) -> crate::device::interface::ImuDiagnostics {
        value.spi_timeouts = TIMEOUTS.load(Ordering::Relaxed);
        value.spi_last_phase = PHASE.load(Ordering::Relaxed);
        value.spi_last_context = LAST_CONTEXT.load(Ordering::Relaxed);
        value.spi_last_elapsed_us = LAST_ELAPSED.load(Ordering::Relaxed);
        value.spi_max_elapsed_us = MAX_ELAPSED.load(Ordering::Relaxed);
        value.spi_last_cmd = LAST_CMD.load(Ordering::Relaxed);
        value.spi_last_clock = LAST_CLOCK.load(Ordering::Relaxed);
        value.spi_last_source = LAST_SOURCE.load(Ordering::Relaxed);
        value.cycle_clock_active = CYCLE_ACTIVE.load(Ordering::Relaxed);
        value.cycle_clock_hz = CYCLE_HZ.load(Ordering::Relaxed);
        value
    }

    fn registers() -> &'static esp32s31::spi2::RegisterBlock {
        // SAFETY: ImuRegisters consumes SPI2 exclusively, with no DMA or ISR.
        unsafe { &*esp32s31::SPI2::ptr() }
    }

    impl<'d> ImuBus<'d> {
        pub(crate) fn delay(&self) -> ImuDelay {
            ImuDelay::new(self.0.clock)
        }
        pub(crate) fn new(
            peripheral: SPI2<'d>,
            sclk: GPIO12<'d>,
            mosi: GPIO11<'d>,
            miso: GPIO13<'d>,
        ) -> Result<Self, ConfigError> {
            let spi = aevia_device_hal::esp32s31::init_imu_spi(peripheral, sclk, mosi, miso)?;
            let clock = CycleClock::prepare()
                .filter(|clock| clock.rate().cycles_for_ns(TRANSFER_US * 1000).is_some());
            let timeout_ticks = clock.map_or(TRANSFER_US, |clock| {
                clock.rate().cycles_for_ns(TRANSFER_US * 1000).unwrap()
            });
            CYCLE_ACTIVE.store(clock.is_some(), Ordering::Relaxed);
            CYCLE_HZ.store(clock.map_or(0, CycleClock::hz), Ordering::Relaxed);
            // SAFETY: SPI2 and its clock are owned by this adapter.
            let clocks = unsafe { &*esp32s31::HP_SYS_CLKRST::ptr() };
            clocks
                .gpspi2_ctrl0()
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
            Ok(Self(ImuRegisters {
                _spi: spi,
                configuration,
                clock,
                timeout_ticks,
            }))
        }
    }

    impl Registers for ImuRegisters<'_> {
        fn now_ticks(&self) -> u32 {
            self.clock.map_or_else(
                || {
                    esp_hal::time::Instant::now()
                        .duration_since_epoch()
                        .as_micros() as u32
                },
                CycleClock::now,
            )
        }
        fn timeout_ticks(&self) -> u32 {
            self.timeout_ticks
        }
        fn micros_for_ticks(&self, ticks: u32) -> u32 {
            self.clock
                .map_or(ticks, |clock| clock.rate().micros_for_cycles(ticks))
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
        fn receive(&self, bytes: &mut [u8]) {
            for (i, chunk) in bytes.chunks_mut(4).enumerate() {
                let word = registers().w(i).read().bits().to_le_bytes();
                chunk.copy_from_slice(&word[..chunk.len()]);
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
            clocks.gpspi2_ctrl0().modify(|_, w| w.rst_en().set_bit());
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            clocks.gpspi2_ctrl0().modify(|_, w| w.rst_en().clear_bit());
            let r = registers();
            unsafe {
                // Reset clears both the register and module clock gates. As
                // in SDK init, enable those clocks before restoring registers.
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
        fn timeout(&mut self, phase: u8, elapsed_us: u32) {
            TIMEOUTS.fetch_add(1, Ordering::Relaxed);
            PHASE.store(phase, Ordering::Relaxed);
            LAST_CONTEXT.store(CONTEXT.load(Ordering::Relaxed), Ordering::Relaxed);
            LAST_ELAPSED.store(elapsed_us, Ordering::Relaxed);
            MAX_ELAPSED.fetch_max(elapsed_us, Ordering::Relaxed);
            LAST_CMD.store(registers().cmd().read().bits(), Ordering::Relaxed);
            LAST_CLOCK.store(registers().clock().read().bits(), Ordering::Relaxed);
            LAST_SOURCE.store(
                unsafe { &*esp32s31::HP_SYS_CLKRST::ptr() }
                    .gpspi2_ctrl0()
                    .read()
                    .bits(),
                Ordering::Relaxed,
            );
        }
        fn completed(&mut self, elapsed_us: u32) {
            MAX_ELAPSED.fetch_max(elapsed_us, Ordering::Relaxed);
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
        fn now_ticks(&self) -> u32 {
            let now = self.time.get();
            self.time.set(now + 1);
            now as u32
        }
        fn busy(&self) -> bool {
            self.busy
        }
        fn updating(&self) -> bool {
            self.stuck_update
        }
        fn load(&mut self, bytes: &[u8]) {
            assert!(!bytes.is_empty() && bytes.len() <= 64);
            self.loaded += bytes.len();
        }
        fn receive(&self, bytes: &mut [u8]) {
            bytes.fill(0x5a);
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
    fn stuck_controller_times_out_before_reading_fifo_and_recovers() {
        for update in [true, false] {
            let mut bus = BoundedBus(Model {
                stuck_update: update,
                stuck_transfer: !update,
                ..Default::default()
            });
            let mut frame = [0x12; 6];
            assert_eq!(bus.transfer_in_place(&mut frame), Err(Error::Timeout));
            assert_eq!(frame, [0x12; 6]);
            assert_eq!(bus.0.resets, 1);
            assert!(bus.0.time.get() <= u64::from(TRANSFER_US) + 2);
            bus.flush().unwrap();
            bus.transfer_in_place(&mut frame).unwrap();
            assert_eq!(frame, [0x5a; 6]);
        }
    }
    #[test]
    fn transfers_cover_partial_fifo_words_and_unequal_lengths() {
        let mut bus = BoundedBus(Model::default());
        let mut read = [0; 67];
        bus.transfer(&mut read, &[1; 70]).unwrap();
        assert_eq!(read, [0x5a; 67]);
        assert_eq!(bus.0.loaded, 70);
        assert_eq!(bus.0.starts, 2);
        bus.transfer(&mut read, &[1; 3]).unwrap();
        assert_eq!(bus.0.loaded, 137);
        assert_eq!(bus.0.starts, 4);
        bus.transfer(&mut [], &[]).unwrap();
        assert_eq!(bus.0.starts, 4);
    }

    #[test]
    fn stuck_transfer_times_out_across_low_counter_wrap() {
        let start = u64::from(u32::MAX) - 50;
        let mut bus = BoundedBus(Model {
            time: Cell::new(start),
            stuck_transfer: true,
            ..Default::default()
        });
        assert_eq!(bus.transfer_in_place(&mut [0; 6]), Err(Error::Timeout));
        assert_eq!(bus.0.resets, 1);
        assert!(bus.0.time.get() > u64::from(u32::MAX));
        assert!(bus.0.time.get() - start <= u64::from(TRANSFER_US) + 2);
        bus.transfer_in_place(&mut [0; 6]).unwrap();
    }

    #[test]
    fn timed_out_sensor_transaction_releases_chip_select_and_can_retry() {
        use aevia_device_hal::{
            bus::ExclusiveSpiDevice,
            embedded_hal::{
                delay::DelayNs,
                digital::{ErrorType, OutputPin},
                spi::{Operation, SpiDevice},
            },
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
        let cs = Cell::new(true);
        let mut device = ExclusiveSpiDevice::new(
            BoundedBus(Model {
                stuck_transfer: true,
                ..Default::default()
            }),
            Pin(&cs),
            Delay,
        );
        device.initialize().unwrap();
        let mut frame = [0; 6];
        assert!(
            device
                .transaction(&mut [Operation::TransferInPlace(&mut frame)])
                .is_err()
        );
        assert!(cs.get());
        device
            .transaction(&mut [Operation::TransferInPlace(&mut frame)])
            .unwrap();
        assert!(cs.get());
        assert_eq!(frame, [0x5a; 6]);
    }

    #[test]
    fn completion_during_preempted_clock_read_does_not_reset_controller() {
        struct Race {
            update: bool,
            sampled_clock: Cell<bool>,
            updating: Cell<bool>,
            busy: Cell<bool>,
            resets: usize,
        }
        impl Registers for Race {
            fn now_ticks(&self) -> u32 {
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
            fn receive(&self, bytes: &mut [u8]) {
                bytes.fill(0x5a);
            }
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
            let mut frame = [0; 6];
            bus.transfer_in_place(&mut frame).unwrap();
            assert_eq!(frame, [0x5a; 6]);
            assert_eq!(bus.0.resets, 0);
        }
    }
}
