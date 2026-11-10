//! Battery-free evaluation policy, shared by application and recovery.
//! Register access, variant checks and power-path-safe writes belong to HAL.
use super::interface::{PowerDiagnostics, PowerFailure, PowerStatus};
use aevia_device_hal::{
    bq25622::{Bq25622, ByteRegister, Error as ChargerError, Variant},
    embedded_hal::i2c::{self, I2c, Operation},
};
use core::cell::Cell;

/// Preserve the backend cause even when embedded-hal maps it to `Other`.
pub(crate) trait DiagnosticError: i2c::Error {
    fn source_code(&self) -> u8;
}

impl DiagnosticError for i2c::ErrorKind {
    fn source_code(&self) -> u8 {
        0
    }
}

impl<E: DiagnosticError> DiagnosticError for aevia_device_hal::bus::I2cError<E> {
    fn source_code(&self) -> u8 {
        match self {
            Self::Bus(error) => error.source_code(),
            Self::Busy => 12,
        }
    }
}

#[cfg(target_arch = "riscv32")]
impl DiagnosticError for esp_hal::i2c::master::Error {
    fn source_code(&self) -> u8 {
        use esp_hal::i2c::master::{AcknowledgeCheckFailedReason, Error::*};
        match self {
            FifoExceeded => 1,
            AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Address) => 2,
            AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Data) => 3,
            AcknowledgeCheckFailed(_) => 4,
            Timeout => 5,
            ArbitrationLost => 6,
            ExecutionIncomplete => 7,
            CommandNumberExceeded => 8,
            ZeroLengthInvalid => 9,
            AddressInvalid(_) => 10,
            _ => 11,
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Transaction {
    register: Option<u8>,
    operation: u8,
    written: Option<u8>,
}

/// Observe HAL transactions without changing their order, contents or retries.
struct ObservedBus<'a, B> {
    bus: B,
    last: &'a Cell<Transaction>,
}
impl<B: I2c> i2c::ErrorType for ObservedBus<'_, B> {
    type Error = B::Error;
}
impl<B: I2c> I2c for ObservedBus<'_, B> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut observed = Transaction::default();
        if let Some(Operation::Write(bytes)) = operations.first() {
            observed.register = bytes.first().copied();
            observed.written = bytes.get(1).copied();
        }
        observed.operation = if operations.iter().any(|op| matches!(op, Operation::Read(_))) {
            1
        } else if operations
            .iter()
            .any(|op| matches!(op, Operation::Write(_)))
        {
            2
        } else {
            0
        };
        self.last.set(observed);
        self.bus.transaction(address, operations)
    }
}

fn bus_error_code(kind: i2c::ErrorKind) -> u8 {
    use i2c::{ErrorKind::*, NoAcknowledgeSource};
    match kind {
        Bus => 1,
        ArbitrationLoss => 2,
        NoAcknowledge(NoAcknowledgeSource::Address) => 3,
        NoAcknowledge(NoAcknowledgeSource::Data) => 4,
        NoAcknowledge(NoAcknowledgeSource::Unknown) => 5,
        Overrun => 6,
        _ => 7,
    }
}

fn failure<E: DiagnosticError>(
    stage: u8,
    error: &ChargerError<E>,
    transaction: Transaction,
    at_us: u64,
) -> PowerFailure {
    let mut value = PowerFailure {
        stage,
        register: transaction.register,
        operation: transaction.operation,
        expected: transaction.written,
        at_us,
        ..Default::default()
    };
    value.error_kind = match error {
        ChargerError::Bus(error) => {
            value.i2c_kind = Some(bus_error_code(error.kind()));
            value.i2c_source = Some(error.source_code());
            1
        }
        ChargerError::Identity { found, .. } => {
            value.expected = Some(3);
            value.actual = Some(found.part_number);
            2
        }
        ChargerError::InvalidValue => 3,
        ChargerError::ReadOnly => 4,
        ChargerError::Unsupported => 5,
        ChargerError::ConversionAborted => 6,
        ChargerError::Verification => 7,
    };
    value
}

fn retain(diagnostics: &mut PowerDiagnostics, stages: &mut u8, failure: PowerFailure) {
    let index = usize::from(failure.stage - 1);
    diagnostics.stage_failures[index] = diagnostics.stage_failures[index].saturating_add(1);
    // Preserve the initiating transport failure when later readbacks also fail
    // or expose a control mismatch in this same attempt.
    if *stages == 0 {
        diagnostics.last_failure = Some(failure);
    }
    *stages |= 1 << index;
}

#[cfg(target_arch = "riscv32")]
static DIAGNOSTICS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Cell<Option<PowerDiagnostics>>,
> = embassy_sync::blocking_mutex::Mutex::new(Cell::new(None));

pub(crate) fn diagnostics() -> PowerDiagnostics {
    #[cfg(target_arch = "riscv32")]
    {
        DIAGNOSTICS.lock(Cell::get).unwrap_or_default()
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        PowerDiagnostics::default()
    }
}

// Include WD_RST in verification: the policy must never issue a watchdog kick.
fn charging_disabled(control: u8) -> u8 {
    control & !0x27
}

/// Apply the bench policy and return only readbacks from this attempt. A failed
/// bus transaction clears unavailable values instead of presenting stale ones.
fn enforce_diagnosed<B: I2c>(
    bus: B,
    previous_errors: u32,
    diagnostics: &mut PowerDiagnostics,
    now: impl Fn() -> u64,
) -> PowerStatus
where
    B::Error: DiagnosticError,
{
    diagnostics.attempts = diagnostics.attempts.saturating_add(1);
    let transaction = Cell::new(Transaction::default());
    let mut charger = Bq25622::new(
        ObservedBus {
            bus,
            last: &transaction,
        },
        Variant::Bq25622E,
    );
    let mut status = PowerStatus::default();
    let mut stages = 0;
    let mut before_control = None;
    let verified = (|| -> Option<bool> {
        status.part_info = match charger.identity() {
            Ok(value) => Some(value.raw),
            Err(error) => {
                retain(
                    diagnostics,
                    &mut stages,
                    failure(1, &error, transaction.get(), now()),
                );
                return None;
            }
        };
        // The HAL verifies the selected silicon before any write. Read control
        // before the policy so we can also verify preservation of power paths.
        let before = match charger.read_byte(ByteRegister::ChargerControl1) {
            Ok(value) => value,
            Err(error) => {
                retain(
                    diagnostics,
                    &mut stages,
                    failure(2, &error, transaction.get(), now()),
                );
                return None;
            }
        };
        before_control = Some(before);
        let result = charger.disable_charging_and_watchdog();
        if let Err(error) = &result {
            retain(
                diagnostics,
                &mut stages,
                failure(3, error, transaction.get(), now()),
            );
        }
        status.control = match charger.read_byte(ByteRegister::ChargerControl1) {
            Ok(value) => Some(value),
            Err(error) => {
                retain(
                    diagnostics,
                    &mut stages,
                    failure(4, &error, transaction.get(), now()),
                );
                None
            }
        };
        status.charger_status = match charger.status() {
            Ok(value) => Some(value.status1),
            Err(error) => {
                retain(
                    diagnostics,
                    &mut stages,
                    failure(5, &error, transaction.get(), now()),
                );
                None
            }
        };
        if let Some(actual) = status.control
            && actual != charging_disabled(before)
        {
            retain(
                diagnostics,
                &mut stages,
                PowerFailure {
                    stage: 6,
                    error_kind: 8,
                    register: Some(ByteRegister::ChargerControl1 as u8),
                    operation: 1,
                    expected: Some(charging_disabled(before)),
                    actual: Some(actual),
                    at_us: now(),
                    ..Default::default()
                },
            );
        }
        Some(result.is_ok() && stages == 0)
    })()
    .unwrap_or(false);
    status.verified = verified;
    status.errors = previous_errors.saturating_add(u32::from(!verified));
    if verified {
        diagnostics.successes = diagnostics.successes.saturating_add(1);
        diagnostics.last_success_us = Some(now());
    } else {
        diagnostics.last_failure_stages = stages;
        diagnostics.failed_part_info = status.part_info;
        diagnostics.failed_pre_control = before_control;
        diagnostics.failed_post_control = status.control;
        diagnostics.failed_charger_status = status.charger_status;
    }
    status
}

#[cfg(test)]
fn enforce<B: I2c>(bus: B, previous_errors: u32) -> PowerStatus
where
    B::Error: DiagnosticError,
{
    enforce_diagnosed(bus, previous_errors, &mut PowerDiagnostics::default(), || 0)
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn maintain<B: I2c>(bus: B)
where
    B::Error: DiagnosticError,
{
    use super::interface;
    let mut diagnostics = self::diagnostics();
    let status = enforce_diagnosed(
        bus,
        interface::power_status().errors,
        &mut diagnostics,
        || embassy_time::Instant::now().as_micros(),
    );
    DIAGNOSTICS.lock(|current| current.set(Some(diagnostics)));
    interface::publish_power(status);
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
        calls: usize,
        fail_at: Option<usize>,
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
            self.calls += 1;
            if self.fail_at == Some(self.calls) {
                return Err(i2c::ErrorKind::NoAcknowledge(
                    i2c::NoAcknowledgeSource::Data,
                ));
            }
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
            calls: 0,
            fail_at: None,
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

    #[test]
    fn every_policy_transaction_retains_stage_register_and_error_after_recovery() {
        for (call, stage, register, operation) in [
            (1, 1, 0x38, 1),
            (2, 2, 0x16, 1),
            (3, 3, 0x38, 1),
            (4, 3, 0x16, 1),
            (5, 3, 0x16, 2),
            (6, 3, 0x16, 1),
            (7, 3, 0x1d, 1),
            (8, 4, 0x16, 1),
            (9, 5, 0x1d, 1),
        ] {
            let mut bus = bus(0xa1);
            bus.fail_at = Some(call);
            let mut diagnostics = PowerDiagnostics::default();
            let status = enforce_diagnosed(&mut bus, 7, &mut diagnostics, || 42);
            assert!(!status.verified, "transaction {call}");
            assert_eq!(status.errors, 8);
            let error = diagnostics.last_failure.unwrap();
            assert_eq!(error.stage, stage, "transaction {call}");
            assert_eq!(error.error_kind, 1);
            assert_eq!(error.i2c_kind, Some(4));
            assert_eq!(error.i2c_source, Some(0));
            assert_eq!(error.register, Some(register));
            assert_eq!(error.operation, operation);
            assert_eq!(error.at_us, 42);
            assert_eq!(diagnostics.stage_failures[stage as usize - 1], 1);
            assert_eq!(diagnostics.attempts, 1);
            assert_eq!(diagnostics.successes, 0);
            let stages = diagnostics.last_failure_stages;
            bus.fail_at = None;
            assert!(enforce_diagnosed(&mut bus, 8, &mut diagnostics, || 99).verified);
            assert_eq!(diagnostics.attempts, 2);
            assert_eq!(diagnostics.successes, 1);
            assert_eq!(diagnostics.last_success_us, Some(99));
            assert_eq!(diagnostics.last_failure.unwrap().at_us, 42);
            assert_eq!(diagnostics.last_failure_stages, stages);
        }
    }

    #[test]
    fn ignored_writes_retain_policy_verification_and_control_mismatch_evidence() {
        let mut bus = bus(0xa1);
        bus.ignore_write = true;
        let mut diagnostics = PowerDiagnostics::default();
        assert!(!enforce_diagnosed(&mut bus, 0, &mut diagnostics, || 7).verified);
        assert_eq!(diagnostics.last_failure_stages, (1 << 2) | (1 << 5));
        assert_eq!(diagnostics.last_failure.unwrap().error_kind, 7);
        assert_eq!(diagnostics.failed_pre_control, Some(0xa1));
        assert_eq!(diagnostics.failed_post_control, Some(0xa1));
        assert_eq!(diagnostics.failed_part_info, Some(24));
        assert_eq!(diagnostics.stage_failures[5], 1);
    }

    #[test]
    fn concrete_timeout_and_shared_bus_busy_remain_distinct_from_portable_other() {
        #[derive(Debug)]
        struct Timeout;
        impl i2c::Error for Timeout {
            fn kind(&self) -> i2c::ErrorKind {
                i2c::ErrorKind::Other
            }
        }
        impl DiagnosticError for Timeout {
            fn source_code(&self) -> u8 {
                5
            }
        }
        let timeout = aevia_device_hal::bus::I2cError::Bus(Timeout);
        let retained = failure(3, &ChargerError::Bus(timeout), Transaction::default(), 10);
        assert_eq!(retained.i2c_kind, Some(7));
        assert_eq!(retained.i2c_source, Some(5));
        let busy = aevia_device_hal::bus::I2cError::<Timeout>::Busy;
        let retained = failure(3, &ChargerError::Bus(busy), Transaction::default(), 11);
        assert_eq!(retained.i2c_kind, Some(7));
        assert_eq!(retained.i2c_source, Some(12));
    }
}
