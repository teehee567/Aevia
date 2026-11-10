//! Bus ownership adapters. Hardware timeouts and bus frequency/mode are set by
//! the MCU backend. Never share these adapters across interrupt contexts.
use core::cell::RefCell;
use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c, spi};

#[derive(Debug, PartialEq, Eq)]
pub enum SpiError<S, P> {
    Bus(S),
    ChipSelect(P),
    NotInitialized,
}
impl<S: spi::Error, P: core::fmt::Debug> spi::Error for SpiError<S, P> {
    fn kind(&self) -> spi::ErrorKind {
        match self {
            Self::Bus(e) => e.kind(),
            _ => spi::ErrorKind::Other,
        }
    }
}

/// Owned, active-low CS device for MCU drivers exposing only `SpiBus`.
/// Configure the underlying bus mode/frequency before construction. Call
/// `initialize()` before the first transaction, with every other CS inactive.
/// The delay object is required for device-level `Operation::DelayNs` support.
pub struct ExclusiveSpiDevice<B, P, D> {
    bus: B,
    cs: P,
    delay: D,
    initialized: bool,
}
impl<B: spi::SpiBus, P: OutputPin, D: DelayNs> ExclusiveSpiDevice<B, P, D> {
    pub const fn new(bus: B, cs: P, delay: D) -> Self {
        Self {
            bus,
            cs,
            delay,
            initialized: false,
        }
    }
    pub fn initialize(&mut self) -> Result<(), SpiError<B::Error, P::Error>> {
        self.initialized = false;
        self.cs.set_high().map_err(SpiError::ChipSelect)?;
        self.initialized = true;
        Ok(())
    }
    pub fn release(self) -> (B, P, D) {
        (self.bus, self.cs, self.delay)
    }
}
impl<B: spi::SpiBus, P: OutputPin, D: DelayNs> spi::ErrorType for ExclusiveSpiDevice<B, P, D> {
    type Error = SpiError<B::Error, P::Error>;
}
impl<B: spi::SpiBus, P: OutputPin, D: DelayNs> spi::SpiDevice for ExclusiveSpiDevice<B, P, D> {
    fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(SpiError::NotInitialized);
        }
        if let Err(error) = self.cs.set_low() {
            // Even an unsuccessful GPIO call may have changed the level.
            let _ = self.cs.set_high();
            self.initialized = false;
            return Err(SpiError::ChipSelect(error));
        }
        let mut result = Ok(());
        for operation in operations {
            let operation_result = match operation {
                spi::Operation::Read(data) => self.bus.read(data),
                spi::Operation::Write(data) => self.bus.write(data),
                spi::Operation::Transfer(read, write) => self.bus.transfer(read, write),
                spi::Operation::TransferInPlace(data) => self.bus.transfer_in_place(data),
                spi::Operation::DelayNs(ns) => self.bus.flush().map(|()| self.delay.delay_ns(*ns)),
            };
            if let Err(error) = operation_result {
                result = Err(SpiError::Bus(error));
                break;
            }
        }
        // Flush and release CS on every exit, including transfer errors.
        let flushed = self.bus.flush().map_err(SpiError::Bus);
        let deselected = self.cs.set_high().map_err(SpiError::ChipSelect);
        if deselected.is_err() {
            self.initialized = false;
        }
        result.and(flushed).and(deselected)
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum I2cError<E> {
    Bus(E),
    Busy,
}
impl<E: i2c::Error> i2c::Error for I2cError<E> {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Self::Bus(e) => e.kind(),
            Self::Busy => i2c::ErrorKind::Other,
        }
    }
}

/// Cooperative single-context I2C sharing without allocation. Each component
/// owns one handle; a transaction takes a nonblocking borrow of the shared bus.
/// Use a vendor/RTOS lock instead when tasks or interrupts can run concurrently.
pub struct SharedI2cDevice<'a, B> {
    bus: &'a RefCell<B>,
}
impl<'a, B> SharedI2cDevice<'a, B> {
    pub const fn new(bus: &'a RefCell<B>) -> Self {
        Self { bus }
    }
}
impl<B: i2c::I2c> i2c::ErrorType for SharedI2cDevice<'_, B> {
    type Error = I2cError<B::Error>;
}
impl<B: i2c::I2c> i2c::I2c for SharedI2cDevice<'_, B> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.bus
            .try_borrow_mut()
            .map_err(|_| I2cError::Busy)?
            .transaction(address, operations)
            .map_err(I2cError::Bus)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use std::{rc::Rc, vec::Vec};
    struct Bus(Rc<RefCell<Vec<&'static str>>>, bool);
    impl spi::ErrorType for Bus {
        type Error = spi::ErrorKind;
    }
    impl spi::SpiBus for Bus {
        fn read(&mut self, _: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
        fn write(&mut self, _: &[u8]) -> Result<(), Self::Error> {
            self.0.borrow_mut().push("write");
            if self.1 {
                Err(spi::ErrorKind::Other)
            } else {
                Ok(())
            }
        }
        fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
        fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            self.0.borrow_mut().push("flush");
            Ok(())
        }
    }
    struct Pin(Rc<RefCell<Vec<&'static str>>>);
    impl embedded_hal::digital::ErrorType for Pin {
        type Error = Infallible;
    }
    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0.borrow_mut().push("low");
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0.borrow_mut().push("high");
            Ok(())
        }
    }
    struct Delay;
    impl DelayNs for Delay {
        fn delay_ns(&mut self, _: u32) {}
    }
    #[test]
    fn failed_spi_transfer_still_flushes_and_releases_cs() {
        use spi::SpiDevice;
        let events = Rc::new(RefCell::new(Vec::new()));
        let mut dev =
            ExclusiveSpiDevice::new(Bus(events.clone(), true), Pin(events.clone()), Delay);
        assert_eq!(dev.write(&[1]), Err(SpiError::NotInitialized));
        assert!(events.borrow().is_empty());
        dev.initialize().unwrap();
        assert_eq!(dev.write(&[1]), Err(SpiError::Bus(spi::ErrorKind::Other)));
        assert_eq!(*events.borrow(), ["high", "low", "write", "flush", "high"]);
    }

    struct I2cBus(usize);
    impl i2c::ErrorType for I2cBus {
        type Error = i2c::ErrorKind;
    }
    impl i2c::I2c for I2cBus {
        fn transaction(&mut self, _: u8, _: &mut [i2c::Operation<'_>]) -> Result<(), Self::Error> {
            self.0 += 1;
            Err(i2c::ErrorKind::ArbitrationLoss)
        }
    }
    #[test]
    fn busy_i2c_never_enters_bus_and_error_releases_shared_borrow() {
        use i2c::I2c;
        let shared = RefCell::new(I2cBus(0));
        let mut first = SharedI2cDevice::new(&shared);
        let mut second = SharedI2cDevice::new(&shared);
        let guard = shared.borrow_mut();
        assert_eq!(first.write(0x6b, &[0x38]), Err(I2cError::Busy));
        assert_eq!(guard.0, 0);
        drop(guard);
        let expected = Err(I2cError::Bus(i2c::ErrorKind::ArbitrationLoss));
        assert_eq!(first.write(0x6b, &[0x38]), expected);
        assert_eq!(second.write(0x36, &[0x02]), expected);
        assert_eq!(shared.borrow().0, 2);
    }
}
