//! Battery-free evaluation policy, shared by application and recovery.
//! Register access, variant checks and power-path-safe writes belong to HAL.
use super::interface::PowerStatus;
use aevia_device_hal::{
    bq25622::{Bq25622, ByteRegister, Variant},
    embedded_hal::i2c::I2c,
};

// Include WD_RST in verification: the policy must never issue a watchdog kick.
fn charging_disabled(control: u8) -> u8 {
    control & !0x27
}

/// Apply the bench policy and return only readbacks from this attempt. A failed
/// bus transaction clears unavailable values instead of presenting stale ones.
fn enforce<B: I2c>(bus: B, previous_errors: u32) -> PowerStatus {
    let mut charger = Bq25622::new(bus, Variant::Bq25622E);
    let mut status = PowerStatus::default();
    let verified = (|| {
        status.part_info = Some(charger.identity().ok()?.raw);
        // The HAL verifies the selected silicon before any write. Read control
        // before the policy so we can also verify preservation of power paths.
        let before = charger.read_byte(ByteRegister::ChargerControl1).ok()?;
        let result = charger.disable_charging_and_watchdog();
        status.control = charger.read_byte(ByteRegister::ChargerControl1).ok();
        status.charger_status = charger.status().ok().map(|s| s.status1);
        result.ok()?;
        Some(status.control == Some(charging_disabled(before)))
    })()
    .unwrap_or(false);
    status.verified = verified;
    status.errors = previous_errors.saturating_add(u32::from(!verified));
    status
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn maintain<B: I2c>(bus: B) {
    use super::interface;
    interface::publish_power(enforce(bus, interface::power_status().errors));
}

#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub(crate) async fn recovery_task(mut bus: esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>) {
    loop {
        maintain(&mut bus);
        embassy_time::Timer::after_secs(1).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aevia_device_hal::embedded_hal::i2c::{self, Operation};

    struct Bus {
        control: u8,
        part: u8,
        writes: usize,
        fail: bool,
        ignore_write: bool,
    }
    impl i2c::ErrorType for Bus {
        type Error = i2c::ErrorKind;
    }
    impl I2c for Bus {
        fn transaction(
            &mut self,
            address: u8,
            ops: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            assert_eq!(address, 0x6b);
            if self.fail {
                return Err(i2c::ErrorKind::Bus);
            }
            match ops {
                [Operation::Write(reg), Operation::Read(out)] => {
                    for (index, value) in out.iter_mut().enumerate() {
                        *value = match reg[0] as usize + index {
                            0x16 => self.control,
                            0x38 => self.part,
                            _ => 0,
                        };
                    }
                }
                [Operation::Write(bytes)] => {
                    assert_eq!(bytes[0], 0x16);
                    self.writes += 1;
                    if !self.ignore_write {
                        self.control = bytes[1];
                    }
                }
                _ => panic!("unexpected transaction"),
            }
            Ok(())
        }
    }
    fn bus(control: u8) -> Bus {
        Bus {
            control,
            part: 3 << 3,
            writes: 0,
            fail: false,
            ignore_write: false,
        }
    }

    #[test]
    fn policy_preserves_power_paths_for_every_control_value() {
        for control in 0..=u8::MAX {
            let mut bus = bus(control);
            let status = enforce(&mut bus, 7);
            assert!(status.verified);
            assert_eq!(status.errors, 7);
            assert_eq!(bus.control, control & !0x27);
            assert_eq!(bus.control & 0xd8, control & 0xd8);
        }
    }

    #[test]
    fn wrong_silicon_never_receives_writes() {
        let mut bus = bus(0xa1);
        bus.part = 1 << 3;
        let status = enforce(&mut bus, 1);
        assert!(!status.verified);
        assert_eq!(status.part_info, Some(8));
        assert_eq!(status.errors, 2);
        assert_eq!(bus.writes, 0);
    }

    #[test]
    fn ignored_write_and_transport_failure_are_observable() {
        let mut bus = bus(0xa1);
        bus.ignore_write = true;
        assert!(!enforce(&mut bus, 0).verified);
        bus.fail = true;
        let status = enforce(&mut bus, u32::MAX);
        assert!(!status.verified);
        assert!(status.control.is_none());
        assert_eq!(status.errors, u32::MAX);
    }

    #[test]
    fn power_query_appends_to_the_existing_device_wire_commands() {
        use crate::{device::interface::DeviceCommand, protocol::Request};
        let mut buffer = [0; 16];
        assert_eq!(
            postcard::to_slice_cobs(&Request::Device(DeviceCommand::Status), &mut buffer).unwrap(),
            &[2, 1, 1, 0]
        );
        assert_eq!(
            postcard::to_slice_cobs(&Request::Device(DeviceCommand::PowerStatus), &mut buffer)
                .unwrap(),
            &[3, 1, 3, 0]
        );
    }
}
