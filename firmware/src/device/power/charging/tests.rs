use super::*;
use std::vec::Vec;

struct Bus {
    registers: [u8; 0x39],
    calls: usize,
    fail_at: Option<usize>,
    ignore_register: Option<u8>,
    corrupt_input_after_limit: bool,
    reset_limits_on_disable: bool,
    writes: Vec<(u8, Vec<u8>)>,
}
impl Bus {
    fn new() -> Self {
        let mut bus = Self {
            registers: [0; 0x39],
            calls: 0,
            fail_at: None,
            ignore_register: None,
            corrupt_input_after_limit: false,
            reset_limits_on_disable: false,
            writes: Vec::new(),
        };
        bus.registers[0x38] = 3 << 3;
        bus.registers[0x14] = 0x06;
        bus.registers[0x15] = 0x0c;
        bus.registers[0x16] = 0xa1;
        bus.registers[0x1a] = 0x0d;
        bus.registers[0x1b] = 0x25;
        bus.registers[0x1c] = 0x31;
        for (reg, raw) in [
            (0x02, 13 << 6),
            (0x04, 420 << 3),
            (0x06, 25 << 4),
            (0x08, 110 << 5),
            (0x0e, 44 << 6),
            (0x10, 5 << 4),
            (0x12, 6 << 3),
        ] {
            bus.set_word(reg, raw);
        }
        bus
    }
    fn set_word(&mut self, register: usize, value: u16) {
        self.registers[register..register + 2].copy_from_slice(&value.to_le_bytes());
    }
    fn word(&self, register: usize) -> u16 {
        u16::from_le_bytes(self.registers[register..register + 2].try_into().unwrap())
    }
    fn run(&mut self) -> (PowerStatus, PowerChargeStatus, PowerDiagnostics) {
        let mut diagnostics = PowerDiagnostics::default();
        let (status, charge) = enforce(&mut *self, 7, &mut diagnostics, || 42);
        (status, charge, diagnostics)
    }
}
impl i2c::ErrorType for Bus {
    type Error = i2c::ErrorKind;
}
impl I2c for Bus {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        assert_eq!(address, 0x6b);
        self.calls += 1;
        if self.fail_at == Some(self.calls) {
            return Err(i2c::ErrorKind::Bus);
        }
        match ops {
            [Operation::Write(reg), Operation::Read(out)] => {
                for (i, byte) in out.iter_mut().enumerate() {
                    let register = reg[0] as usize + i;
                    *byte = if register == 0x1e {
                        // Charge state becomes idle after EN_CHG is cleared.
                        4 | if self.registers[0x16] & 0x20 != 0 {
                            8
                        } else {
                            0
                        }
                    } else {
                        self.registers[register]
                    };
                }
            }
            [Operation::Write(bytes)] => {
                let reg = bytes[0];
                assert!(
                    matches!(reg, 0x02 | 0x04 | 0x10 | 0x12 | 0x16),
                    "unrelated register written: {reg:x}"
                );
                if reg != 0x16 {
                    assert_eq!(
                        self.registers[0x16] & 0x20,
                        0,
                        "limit written while charging"
                    );
                }
                self.writes.push((reg, bytes[1..].to_vec()));
                if self.ignore_register != Some(reg) {
                    let start = reg as usize;
                    self.registers[start..start + bytes.len() - 1].copy_from_slice(&bytes[1..]);
                }
                if reg == 0x16 && bytes[1] & 0x23 == 0 && self.reset_limits_on_disable {
                    // A watchdog expiry immediately before the disable write
                    // restored the default current after the initial reads.
                    self.set_word(0x02, 13 << 6);
                    self.reset_limits_on_disable = false;
                }
                if reg != 0x16 && self.corrupt_input_after_limit {
                    self.registers[0x06] ^= 0x10;
                    self.corrupt_input_after_limit = false;
                }
            }
            _ => panic!("unexpected transaction"),
        }
        Ok(())
    }
}

#[test]
fn supervised_profile_has_exact_readbacks_and_preserves_all_other_registers() {
    let mut bus = Bus::new();
    let before = bus.registers;
    let (status, charge, diagnostics) = bus.run();
    assert!(status.verified);
    assert_eq!(status.errors, 7);
    assert!(charge.verified);
    assert_eq!(charge.charging_enabled, Some(true));
    assert_eq!(charge.charge_current_ma, Some(800));
    assert_eq!(charge.charge_voltage_mv, Some(4100));
    assert_eq!(charge.precharge_current_ma, Some(80));
    assert_eq!(charge.termination_current_ma, Some(40));
    assert_eq!(charge.input_current_limit_ma, Some(500));
    assert_eq!(charge.charger_status1, Some(12));
    assert_eq!(charge.fault_status, Some(0));
    assert_eq!(charge.updated_us, Some(42));
    assert_eq!(diagnostics.successes, 1);
    for (i, &value) in before.iter().enumerate() {
        if !matches!(i, 0x02..=0x05 | 0x10..=0x13 | 0x16) {
            assert_eq!(bus.registers[i], value, "register {i:x}");
        }
    }
    assert_eq!(bus.registers[0x16], 0xa0);
    assert_eq!(bus.word(0x02), 0x0280);
    assert_eq!(bus.word(0x04), 0x0cd0);
    assert_eq!(bus.word(0x10), 0x0040);
    assert_eq!(bus.word(0x12), 0x0020);
    let writes = bus.writes.len();
    assert!(bus.run().0.verified);
    assert_eq!(
        bus.writes.len(),
        writes,
        "periodic verification must not restart charging"
    );
}

#[test]
fn every_transport_failure_disables_charging_and_retains_the_initiating_cause() {
    let mut good = Bus::new();
    assert!(good.run().0.verified);
    for fail_at in 1..=good.calls {
        let mut bus = Bus::new();
        bus.fail_at = Some(fail_at);
        let (status, charge, diagnostics) = bus.run();
        assert!(!status.verified, "call {fail_at}");
        assert_eq!(status.errors, 8);
        assert!(!charge.verified);
        assert_eq!(bus.registers[0x16] & 0x27, 0, "call {fail_at}");
        assert_eq!(charge.charging_enabled, Some(false));
        let failure = diagnostics.last_failure.unwrap();
        assert_eq!(failure.error_kind, 1, "call {fail_at}");
        assert_eq!(failure.at_us, 42);
        assert_eq!(diagnostics.successes, 0);
    }
}

#[test]
fn ignored_limit_writes_and_preserved_input_corruption_cannot_enable_charging() {
    for register in [0x02, 0x04, 0x10, 0x12] {
        let mut bus = Bus::new();
        bus.ignore_register = Some(register);
        let (status, charge, diagnostics) = bus.run();
        assert!(!status.verified);
        assert_eq!(charge.charging_enabled, Some(false));
        assert_eq!(diagnostics.last_failure.unwrap().register, Some(register));
        assert!(
            !bus.writes
                .iter()
                .any(|(reg, data)| *reg == 0x16 && data[0] & 0x20 != 0)
        );
    }
    let mut bus = Bus::new();
    bus.corrupt_input_after_limit = true;
    let (status, charge, diagnostics) = bus.run();
    assert!(!status.verified);
    assert_eq!(charge.charging_enabled, Some(false));
    assert_eq!(diagnostics.last_failure.unwrap().register, Some(0x06));
}

#[test]
fn protections_must_be_active_and_are_never_rewritten() {
    for (register, value) in [(0x14, 0x46), (0x14, 0x02), (0x15, 0x08), (0x1a, 0x8d)] {
        let mut bus = Bus::new();
        bus.registers[register] = value;
        let (status, charge, diagnostics) = bus.run();
        assert!(!status.verified);
        assert_eq!(charge.charging_enabled, Some(false));
        assert_eq!(
            diagnostics.last_failure.unwrap().register,
            Some(register as u8)
        );
        assert_eq!(bus.registers[register], value);
        assert!(bus.writes.iter().all(|(reg, _)| *reg == 0x16));
    }
}

#[test]
fn wrong_silicon_never_receives_a_write_and_failed_disable_is_not_hidden() {
    let mut bus = Bus::new();
    bus.registers[0x38] = 1 << 3;
    assert!(!bus.run().0.verified);
    assert!(bus.writes.is_empty());
    let mut bus = Bus::new();
    bus.ignore_register = Some(0x16);
    let (status, charge, _) = bus.run();
    assert!(!status.verified);
    assert_eq!(charge.charging_enabled, Some(true));
    assert_eq!(charge.charger_control1, Some(0xa1));
    assert!(bus.writes.iter().all(|(reg, _)| *reg == 0x16));
}

#[test]
fn configuration_verification_does_not_claim_current_flow_or_hide_faults() {
    let mut bus = Bus::new();
    bus.registers[0x1d] = 0x0c; // input voltage/current regulation
    bus.registers[0x1f] = 0x02; // hot temperature zone
    let (_, charge, _) = bus.run();
    assert!(charge.verified);
    assert_eq!(charge.charger_status0, Some(0x0c));
    assert_eq!(charge.fault_status, Some(2));
}

#[test]
fn stale_future_and_missing_charge_snapshots_are_not_live_readbacks() {
    let mut bus = Bus::new();
    let (_, charge, _) = bus.run();
    assert!(fresh_status(charge, 3_000_042).verified);
    for now in [41, 3_000_043, u64::MAX] {
        let stale = fresh_status(charge, now);
        assert!(!stale.verified);
        assert_eq!(stale.updated_us, Some(42));
        assert_eq!(stale.charging_enabled, None);
        assert_eq!(stale.charge_current_ma, None);
        assert_eq!(stale.fault_status, None);
    }
    assert!(!fresh_status(PowerChargeStatus::default(), 0).verified);
}

#[test]
fn matching_limits_with_active_watchdog_are_reverified_before_enabling() {
    let mut bus = Bus::new();
    for (index, _, raw) in LIMITS {
        bus.set_word(WORDS[index] as usize, raw);
    }
    bus.reset_limits_on_disable = true;
    let (status, charge, diagnostics) = bus.run();
    assert!(!status.verified);
    assert_eq!(charge.charging_enabled, Some(false));
    assert_eq!(charge.charge_current_ma, Some(1040));
    assert_eq!(diagnostics.last_failure.unwrap().register, Some(0x02));
    assert!(
        !bus.writes
            .iter()
            .any(|(reg, data)| *reg == 0x16 && data[0] & 0x20 != 0)
    );
    assert!(
        bus.run().0.verified,
        "a later attempt safely repairs the reset profile"
    );
}
