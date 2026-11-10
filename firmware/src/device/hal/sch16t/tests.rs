use super::*;
use embedded_hal::spi::{Error as _, ErrorKind, ErrorType};
use std::vec::Vec;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BusError;
impl embedded_hal::spi::Error for BusError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Register-level device simulation. Responses are computed when a command
/// arrives and retired on the following CS assertion, as the datasheet requires.
struct Bus {
    registers: [u16; 64],
    sensor: [i32; 16],
    pending_response: [u8; 6],
    transfers: Vec<[u8; 6]>,
    fault_at: Option<usize>,
    corrupt_at: Option<usize>,
    sensor_status: u8,
    startup_transition_fault: bool,
    persistent_common_fault: bool,
    ignored_configuration_write: Option<Register>,
}

impl Bus {
    fn new() -> Self {
        let mut s = Self {
            registers: [0; 64],
            sensor: [0; 16],
            pending_response: [0; 6],
            transfers: Vec::new(),
            fault_at: None,
            corrupt_at: None,
            sensor_status: 0,
            startup_transition_fault: false,
            persistent_common_fault: false,
            ignored_configuration_write: None,
        };
        s.reset();
        s
    }

    fn reset(&mut self) {
        self.registers = [0; 64];
        for r in STATUS_REGISTERS {
            self.registers[r.address() as usize] = 0xffff;
        }
        self.registers[Register::ComponentId.address() as usize] = 0x23;
        self.registers[Register::AsicId.address() as usize] = 0x10;
    }

    fn accept(&mut self, request: [u8; 6]) {
        let frame = u64::from_be_bytes([
            0, 0, request[0], request[1], request[2], request[3], request[4], request[5],
        ]);
        assert_eq!(crc8(frame), frame as u8);
        let address = ((frame >> 38) & 0x3ff) as usize;
        assert!(address < 64);
        let write = frame & (1 << 37) != 0;
        let value = ((frame >> 8) & 0xffff) as u16;
        let sensor = (1..=0x10).contains(&address) && !write;
        let mut response = (address as u64) << 37;
        if sensor {
            response |= 1 << 47;
            response |= ((self.sensor[address - 1] as u32 & 0xfffff) as u64) << 8;
            response |= u64::from(self.sensor_status) << 33;
            response |= 15 << 29;
        } else {
            response |= u64::from(self.registers[address]) << 8;
            if self.startup_transition_fault
                && self.registers[Register::StatusCommon.address() as usize] != 0xffff
            {
                response |= (1 << 36) | (1 << 33);
            }
        }
        if write {
            if address == Register::Reset.address() as usize {
                self.reset();
            } else {
                if self
                    .ignored_configuration_write
                    .map(|register| register.address() as usize)
                    != Some(address)
                {
                    self.registers[address] = value;
                }
                if address == Register::Mode.address() as usize
                    && value == 3
                    && self.startup_transition_fault
                {
                    self.registers[Register::StatusSummary.address() as usize] = 0xff7f;
                    self.registers[Register::StatusCommon.address() as usize] = 0xfef7;
                }
            }
        } else if address == Register::StatusCommon.address() as usize
            && self.startup_transition_fault
            && !self.persistent_common_fault
        {
            for r in STATUS_REGISTERS {
                self.registers[r.address() as usize] = 0xffff;
            }
        }
        response |= u64::from(crc8(response));
        let b = response.to_be_bytes();
        self.pending_response = [b[2], b[3], b[4], b[5], b[6], b[7]];
    }
}

#[test]
fn startup_acknowledges_latched_mode_transition_then_checks_for_persistent_faults() {
    let mut bus = Bus::new();
    bus.startup_transition_fault = true;
    let mut imu = Sch16t::new(bus, Delay::default());
    imu.initialize(Config::default()).unwrap();
    let (_, delay) = imu.release();
    assert!(delay.0.iter().filter(|&&ns| ns == 3_000_000).count() >= 2);

    let mut bus = Bus::new();
    bus.startup_transition_fault = true;
    bus.persistent_common_fault = true;
    let mut imu = Sch16t::new(bus, Delay::default());
    let error = imu.initialize(Config::default()).unwrap_err();
    let Error::StartupStatus(status) = error else {
        panic!("persistent fault must stay visible")
    };
    assert_eq!(status.registers[0], 0xff7f);
    assert_eq!(status.registers[2], 0xfef7);
    assert!(!status.healthy());
    assert!(imu.configuration().is_none());
}

#[test]
fn ignored_enable_is_reported_as_readback_failure_before_secondary_status_faults() {
    let mut bus = Bus::new();
    bus.startup_transition_fault = true;
    bus.persistent_common_fault = true;
    bus.ignored_configuration_write = Some(Register::Mode);
    let mut imu = Sch16t::new(bus, Delay::default());
    assert!(matches!(
        imu.initialize(Config::default()),
        Err(Error::Readback {
            register: Register::Mode,
            expected: 3,
            actual: 0
        })
    ));
    assert!(imu.configuration().is_none());
}

impl ErrorType for Bus {
    type Error = BusError;
}
impl SpiDevice<u8> for Bus {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        assert_eq!(operations.len(), 3);
        assert!(matches!(operations[0], Operation::DelayNs(1000)));
        assert!(matches!(operations[2], Operation::DelayNs(1000)));
        let Operation::TransferInPlace(wire) = &mut operations[1] else {
            panic!("single SafeSPI frame required");
        };
        assert_eq!(wire.len(), 6);
        let request: [u8; 6] = wire[..].try_into().unwrap();
        self.transfers.push(request);
        if self.fault_at == Some(self.transfers.len()) {
            return Err(BusError);
        }
        wire.copy_from_slice(&self.pending_response);
        if self.corrupt_at == Some(self.transfers.len()) {
            wire[3] ^= 1;
        }
        self.accept(request);
        Ok(())
    }
}

#[derive(Default)]
struct Delay(Vec<u64>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(u64::from(ns));
    }
    fn delay_ms(&mut self, ms: u32) {
        self.0.push(u64::from(ms) * 1_000_000);
    }
}

#[test]
fn startup_verifies_then_locks_config_and_reads_full_resolution() {
    let mut bus = Bus::new();
    bus.sensor[0] = -512;
    bus.sensor[4] = 3520;
    bus.sensor[15] = -3000 * 16;
    let mut imu = Sch16t::new(bus, Delay::default());
    let identity = imu.initialize(Config::default()).unwrap();
    assert_eq!(identity.component, 0x23);
    let sample = imu.motion(MotionOutput::Interpolated).unwrap();
    assert!((sample.angular_rate_dps[0] + 0.32).abs() < 1e-12);
    assert!((sample.acceleration_mps2[1] - 1.1).abs() < 1e-12);
    assert_eq!(imu.temperature_celsius().unwrap(), -30.0);
    assert!(matches!(
        imu.write_register(Register::UserInterface, 0x200c),
        Err(Error::ConfigurationLocked)
    ));
    imu.write_register(Register::SystemTest, 0xabcd).unwrap();
    assert_eq!(
        imu.read_register(Register::SystemTest)
            .unwrap()
            .information(),
        0xabcd
    );
    let (_, delay) = imu.release();
    for ns in [2_000_000, 32_000_000, 215_000_000, 3_000_000] {
        assert!(delay.0.contains(&ns));
    }
}

#[test]
fn invalid_configuration_does_not_touch_bus() {
    let mut imu = Sch16t::new(Bus::new(), Delay::default());
    let mut config = Config::default();
    config.interface.data_ready_enabled = true;
    config.interface.sync_decimated = true;
    assert!(matches!(
        imu.initialize(config),
        Err(Error::Configuration(
            ConfigError::ConflictingSyncAndDataReady
        ))
    ));
    assert!(matches!(
        imu.write_register(Register::ComponentId, 1),
        Err(Error::ReadOnly(Register::ComponentId))
    ));
    assert!(matches!(
        imu.write_register(Register::SelfTest, 0),
        Err(Error::InvalidRegisterValue(Register::SelfTest))
    ));
    assert!(imu.release().0.transfers.is_empty());
}

#[test]
fn six_axis_burst_is_drained_after_bad_crc() {
    let mut bus = Bus::new();
    bus.corrupt_at = Some(3);
    let mut imu = Sch16t::new(bus, Delay::default());
    assert!(matches!(
        imu.read_registers([
            Register::RateX2,
            Register::RateY2,
            Register::RateZ2,
            Register::AccX2,
            Register::AccY2,
            Register::AccZ2,
        ]),
        Err(Error::Frame(FrameError::Crc))
    ));
    assert_eq!(imu.release().0.transfers.len(), 7);
}

#[test]
fn transport_error_discards_ambiguous_response_and_recovers() {
    let mut bus = Bus::new();
    bus.fault_at = Some(2);
    let mut imu = Sch16t::new(bus, Delay::default());
    assert!(matches!(
        imu.read_register(Register::ComponentId),
        Err(Error::Spi(BusError))
    ));
    assert_eq!(
        imu.read_register(Register::ComponentId)
            .unwrap()
            .information(),
        0x23
    );
}

#[test]
fn non_normal_sensor_metadata_rejects_temperature_and_motion() {
    let mut bus = Bus::new();
    bus.sensor_status = 2;
    let mut imu = Sch16t::new(bus, Delay::default());
    imu.initialize(Config::default()).unwrap();
    assert!(matches!(
        imu.temperature_celsius(),
        Err(Error::SensorStatus {
            status: SensorStatus::Saturation,
            ..
        })
    ));
    assert!(matches!(
        imu.motion(MotionOutput::Decimated),
        Err(Error::SensorStatus {
            status: SensorStatus::Saturation,
            ..
        })
    ));
}

#[test]
fn rejected_sensor_status_preserves_the_exact_payload_and_counter() {
    let mut bus = Bus::new();
    bus.sensor_status = 2;
    bus.sensor[Register::RateX2.address() as usize - 1] = -123_456;
    let mut imu = Sch16t::new(bus, Delay::default());
    imu.initialize(Config::default()).unwrap();
    let before = imu.spi.transfers.len();
    assert!(matches!(
        imu.motion(MotionOutput::Decimated),
        Err(Error::SensorStatus {
            register: Register::RateX2,
            status: SensorStatus::Saturation,
            common_error: false,
            payload: 0x0e1dc0,
            counter: 15,
        })
    ));
    // Reporting a rejected axis must still drain the complete pipelined burst.
    assert_eq!(imu.spi.transfers.len() - before, 7);
}

#[test]
fn counter_wrap_is_explicit() {
    let old = MotionSample {
        raw_rate: [0; 3],
        raw_acceleration: [0; 3],
        angular_rate_dps: [0.0; 3],
        angular_rate_rps: [0.0; 3],
        acceleration_mps2: [0.0; 3],
        counters: [15; 6],
    };
    let new = MotionSample {
        counters: [0, 1, 3, 15, 0, 0],
        ..old
    };
    assert_eq!(new.counter_advance(&old), [1, 2, 4, 0, 1, 1]);
    assert_eq!(BusError.kind(), ErrorKind::Other);
}
