use super::*;
use std::vec::Vec;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MockError {
    ResponseTimeout,
    Crc,
    Removed,
}
#[derive(Clone, Debug, Eq, PartialEq)]
enum Event {
    Bus(BusWidth, u32),
    Clocks,
    Command(Command),
    Read(u8, u32, usize),
    Write(u8, u32, u8),
}
struct Mock {
    events: Vec<Event>,
    high_capacity: bool,
    legacy: bool,
    present: bool,
    protected: bool,
    csd: [u32; 4],
    scr: [u8; 8],
    ocr_wait: u16,
    ready_wait: u16,
    fail_command: Option<(u8, MockError)>,
    fail_read: bool,
    data_status: u32,
    app_accepted: bool,
}
fn words(raw: u128) -> [u32; 4] {
    core::array::from_fn(|i| (raw >> (i * 32)) as u32)
}
fn high_csd(c_size: u32) -> [u32; 4] {
    words(1 << 126 | 0x32 << 96 | 0x5b5 << 84 | 9 << 80 | u128::from(c_size) << 48 | 9 << 22)
}
fn standard_csd(read_block_log: u8) -> [u32; 4] {
    // 1024 native blocks of 1024B when READ_BL_LEN10;2048 logical sectors.
    words(0x32 << 96 | 0x5b5 << 84 | u128::from(read_block_log) << 80 | 255 << 62 | 9 << 22)
}
fn ready() -> u32 {
    READY | 4 << 9
}
impl Mock {
    fn high() -> Self {
        Self {
            events: Vec::new(),
            high_capacity: true,
            legacy: false,
            present: true,
            protected: false,
            csd: high_csd(0x1234),
            scr: (5_u64 << 48).to_be_bytes(),
            ocr_wait: 0,
            ready_wait: 0,
            fail_command: None,
            fail_read: false,
            data_status: ready(),
            app_accepted: true,
        }
    }
    fn standard(legacy: bool) -> Self {
        Self {
            high_capacity: false,
            legacy,
            csd: standard_csd(10),
            ..Self::high()
        }
    }
}
impl NativeHost for Mock {
    type Error = MockError;
    fn set_bus(&mut self, width: BusWidth, hz: u32) -> Result<(), Self::Error> {
        self.events.push(Event::Bus(width, hz));
        Ok(())
    }
    fn send_initial_clocks(&mut self) -> Result<(), Self::Error> {
        self.events.push(Event::Clocks);
        Ok(())
    }
    fn command(&mut self, command: Command) -> Result<[u32; 4], Self::Error> {
        self.events.push(Event::Command(command));
        if let Some((index, error)) = self.fail_command
            && command.index == index
        {
            return Err(error);
        }
        let mut response = [0; 4];
        response[0] = match command.index {
            0 => 0,
            8 if self.legacy => return Err(MockError::ResponseTimeout),
            8 => 0x1aa,
            55 => {
                if self.app_accepted {
                    APP_CMD
                } else {
                    0
                }
            }
            41 => {
                assert!(!command.check_crc);
                assert_eq!(command.argument & (1 << 30) != 0, !self.legacy);
                if self.ocr_wait > 0 {
                    self.ocr_wait -= 1;
                    VOLTAGE_WINDOW
                } else {
                    VOLTAGE_WINDOW | 1 << 31 | if self.high_capacity { 1 << 30 } else { 0 }
                }
            }
            2 => return Ok([0x1234, 0x5678, 0x90ab, 0xcdef]),
            3 => 0x1234 << 16 | READY | 3 << 9,
            9 => return Ok(self.csd),
            7 => {
                assert!(command.wait_busy);
                ready()
            }
            13 => {
                assert_eq!(command.argument, 0x1234 << 16);
                if self.ready_wait > 0 {
                    self.ready_wait -= 1;
                    7 << 9
                } else {
                    ready()
                }
            }
            6 => {
                assert!(matches!(command.argument, 0 | 2));
                ready()
            }
            16 => {
                assert_eq!(command.argument, 512);
                ready()
            }
            other => panic!("unexpected command{other}"),
        };
        Ok(response)
    }
    fn read_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &mut [u8],
    ) -> Result<u32, Self::Error> {
        self.events.push(Event::Read(command, argument, data.len()));
        if self.fail_read {
            return Err(MockError::Crc);
        }
        match command {
            51 => {
                assert_eq!(argument, 0);
                data.copy_from_slice(&self.scr);
            }
            17 => data.fill(0x6b),
            other => panic!("unexpected data command{other}"),
        }
        Ok(self.data_status)
    }
    fn write_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &[u8; 512],
    ) -> Result<u32, Self::Error> {
        assert_eq!(command, 24);
        self.events.push(Event::Write(command, argument, data[0]));
        Ok(self.data_status)
    }
    fn card_present(&self) -> bool {
        self.present
    }
    fn write_protected(&self) -> bool {
        self.protected
    }
    fn is_response_timeout(error: &Self::Error) -> bool {
        *error == MockError::ResponseTimeout
    }
}
#[derive(Default)]
struct Delay {
    milliseconds: u32,
}
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.milliseconds += ns / 1_000_000;
    }
}
fn initialized(host: Mock) -> SdCard<Mock, Delay> {
    let mut card = SdCard::new(host, Delay::default());
    card.initialize(Config::default()).unwrap();
    card
}

#[test]
fn initialization_sets_default_clock_reads_registers_and_checks_four_bit_support() {
    let mut host = Mock::high();
    host.ocr_wait = 2;
    let mut card = SdCard::new(host, Delay::default());
    assert!(card.host.events.is_empty());
    assert_eq!(card.sector_count(), 0);
    let info = card.initialize(Config::default()).unwrap();
    assert_eq!(info.sector_count, (0x1234 + 1) * 1024);
    assert!(info.block_addressed);
    assert_eq!(info.relative_address, 0x1234);
    assert_eq!(info.bus_width, BusWidth::Four);
    assert_eq!(
        card.host.events[..2],
        [Event::Bus(BusWidth::One, 400_000), Event::Clocks]
    );
    assert!(card.host.events.contains(&Event::Read(51, 0, 8)));
    assert!(
        card.host
            .events
            .contains(&Event::Bus(BusWidth::Four, 25_000_000))
    );
    assert!(
        !card
            .host
            .events
            .iter()
            .any(|e| matches!(e,Event::Command(c) if c.index==16))
    );
    assert_eq!(card.delay.milliseconds, 3);
}

#[test]
fn sdsc_capacity_includes_native_block_length_and_addresses_bytes() {
    for legacy in [false, true] {
        let mut card = initialized(Mock::standard(legacy));
        assert_eq!(card.sector_count(), 2048);
        assert!(!card.info().unwrap().block_addressed);
        assert!(
            card.host
                .events
                .iter()
                .any(|e| matches!(e,Event::Command(c) if c.index==16&&c.argument==512))
        );
        let mut data = [0; 512];
        card.read_sector(3, &mut data).unwrap();
        assert_eq!(data, [0x6b; 512]);
        card.write_sector(4, &[7; 512]).unwrap();
        assert!(card.host.events.contains(&Event::Read(17, 3 * 512, 512)));
        assert!(card.host.events.contains(&Event::Write(24, 4 * 512, 7)));
    }
}

#[test]
fn high_capacity_addresses_sectors_and_accepts_the_last_two_tb_sector() {
    let mut mock = Mock::high();
    mock.csd = high_csd(0x3f_ffff);
    let mut card = initialized(mock);
    assert_eq!(card.sector_count(), 1_u64 << 32);
    card.write_sector(u64::from(u32::MAX), &[1; 512]).unwrap();
    assert!(card.host.events.contains(&Event::Write(24, u32::MAX, 1)));
    let count = card.host.events.len();
    assert_eq!(
        card.read_sector(1_u64 << 32, &mut [0; 512]),
        Err(Error::OutOfRange)
    );
    assert_eq!(card.host.events.len(), count);
}

#[test]
fn one_bit_only_scr_falls_back_and_card_speed_limits_clock() {
    let mut mock = Mock::standard(false);
    mock.scr = (1_u64 << 48).to_be_bytes();
    let mut raw = u128::from(mock.csd[0])
        | u128::from(mock.csd[1]) << 32
        | u128::from(mock.csd[2]) << 64
        | u128::from(mock.csd[3]) << 96;
    raw = (raw & !(0xff << 96)) | 0x0a << 96; //10MHz
    mock.csd = words(raw);
    let card = initialized(mock);
    assert_eq!(card.info().unwrap().bus_width, BusWidth::One);
    assert_eq!(card.info().unwrap().clock_hz, 10_000_000);
}

#[test]
fn invalid_config_and_out_of_range_have_no_io_and_do_not_destroy_valid_state() {
    let mut card = SdCard::new(Mock::high(), Delay::default());
    assert_eq!(
        card.initialize(Config {
            clock_hz: 50_000_000,
            ..Config::default()
        }),
        Err(Error::InvalidConfiguration)
    );
    assert!(card.host.events.is_empty());
    card.initialize(Config::default()).unwrap();
    let count = card.host.events.len();
    assert_eq!(
        card.read_sector(u64::MAX, &mut [0; 512]),
        Err(Error::OutOfRange)
    );
    assert_eq!(card.host.events.len(), count);
    assert!(card.info().is_some());
}

#[test]
fn initialization_and_flush_polling_are_bounded() {
    let mut mock = Mock::high();
    mock.ocr_wait = 100;
    let mut card = SdCard::new(mock, Delay::default());
    assert_eq!(
        card.initialize(Config {
            initialization_attempts: 3,
            ..Config::default()
        }),
        Err(Error::InitializationTimeout)
    );
    assert!(card.info().is_none());
    assert_eq!(
        card.host
            .events
            .iter()
            .filter(|e| matches!(e,Event::Command(c) if c.index==41))
            .count(),
        3
    );
    let mut card = initialized(Mock::high());
    card.ready_attempts = 3;
    card.host.ready_wait = 100;
    let before = card.host.events.len();
    assert_eq!(card.flush(), Err(Error::ReadyTimeout));
    assert_eq!(card.host.events.len() - before, 3);
    assert!(card.info().is_none());
}

#[test]
fn crc_errors_and_card_removal_invalidate_cached_geometry() {
    let mut card = initialized(Mock::high());
    card.host.fail_read = true;
    assert_eq!(
        card.read_sector(0, &mut [0; 512]),
        Err(Error::Host(MockError::Crc))
    );
    assert!(card.info().is_none());
    assert_eq!(card.flush(), Err(Error::NotInitialized));
    let mut card = initialized(Mock::high());
    card.host.present = false;
    assert_eq!(card.flush(), Err(Error::NoCard));
    assert_eq!(card.sector_count(), 0);
}

#[test]
fn r1_errors_including_out_of_range_are_never_accepted_as_success() {
    for bit in [
        31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 16, 15, 13, 3,
    ] {
        let mut card = initialized(Mock::high());
        card.host.data_status = 1 << bit;
        assert_eq!(
            card.read_sector(0, &mut [0; 512]),
            Err(Error::CardStatus(1 << bit))
        );
        assert!(card.info().is_none());
    }
}

#[test]
fn physical_or_csd_write_protection_prevents_write_before_any_command() {
    let mut card = initialized(Mock::high());
    card.host.protected = true;
    let count = card.host.events.len();
    assert_eq!(card.write_sector(0, &[0; 512]), Err(Error::WriteProtected));
    assert_eq!(card.host.events.len(), count);
    let mut mock = Mock::high();
    mock.csd[0] |= 1 << 13;
    let mut card = initialized(mock);
    assert!(card.info().unwrap().write_protected);
    assert_eq!(card.write_sector(0, &[0; 512]), Err(Error::WriteProtected));
}

#[test]
fn cmd8_crc_is_not_mistaken_for_a_legacy_card_and_app_prefix_is_checked() {
    let mut mock = Mock::high();
    mock.fail_command = Some((8, MockError::Crc));
    let mut card = SdCard::new(mock, Delay::default());
    assert_eq!(
        card.initialize(Config::default()),
        Err(Error::Host(MockError::Crc))
    );
    assert!(
        !card
            .host
            .events
            .iter()
            .any(|e| matches!(e,Event::Command(c) if c.index==41))
    );
    let mut mock = Mock::high();
    mock.app_accepted = false;
    let mut card = SdCard::new(mock, Delay::default());
    assert_eq!(
        card.initialize(Config::default()),
        Err(Error::ApplicationCommand)
    );
}

#[test]
fn malformed_csd_and_scr_are_rejected_before_sector_access() {
    let mut mock = Mock::high();
    mock.csd[3] |= 1 << 31; //CSDv3/SDUC unsupported.
    let mut card = SdCard::new(mock, Delay::default());
    assert_eq!(
        card.initialize(Config::default()),
        Err(Error::UnsupportedCard)
    );
    let mut mock = Mock::high();
    mock.scr = [0; 8];
    let mut card = SdCard::new(mock, Delay::default());
    assert_eq!(card.initialize(Config::default()), Err(Error::InvalidScr));
    assert!(card.info().is_none());
}

#[test]
fn status_and_flush_wait_until_programming_finishes_and_preserve_host_error() {
    let mut card = initialized(Mock::high());
    card.host.ready_wait = 2;
    let elapsed = card.delay.milliseconds;
    card.flush().unwrap();
    assert_eq!(card.delay.milliseconds - elapsed, 2);
    assert_eq!(card.status().unwrap(), Status { raw: ready() });
    card.host.fail_command = Some((13, MockError::Removed));
    assert_eq!(card.status(), Err(Error::Host(MockError::Removed)));
    assert!(card.info().is_none());
}

#[test]
fn all_transfer_speed_codes_are_checked_without_integer_overflow() {
    let baseline = high_csd(0x1234);
    for speed in 0..=255_u32 {
        let mut csd = baseline;
        csd[3] = (csd[3] & !0xff) | speed;
        let geometry = Geometry::parse(csd, true);
        let expected_valid = speed & 0x80 == 0 && speed & 7 <= 3 && speed & 0x78 != 0;
        if !expected_valid {
            assert!(geometry.is_err());
        }
        if let Ok(geometry) = geometry {
            assert!((400_000..=25_000_000).contains(&geometry.maximum_clock_hz));
        }
    }
}
