#![no_std]
#![no_main]

use core::fmt::Write as _;

use aevia_firmware::peripheral_probe::{
    SCH16T_READ_COMPONENT_ID, SCH16T_SOFT_RESET, is_um980_connection_response,
    is_um980_version_reply, parse_um980_gga, sch16t_component_id, sch16t_frame,
    sch16t_frame_has_valid_crc, sch16t_request_bytes,
};
use aevia_firmware::power_bus::{
    read_bq2562x, read_lp5813a_reset_state, read_max17048, read_tca9536a,
};
use aligned::{A4, Aligned};
use block_device_driver::BlockDevice as BlockDeviceDriver;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    sdmmc::{Config as SdConfig, SdHostController, SlotConfig},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, RxConfig, Uart},
    usb::otg::{
        Usb,
        embassy_usb_device::{Config as UsbDriverConfig, Driver},
    },
};
use sdio::{BlockDevice as SdBlockDevice, sd::Card};

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! usb_line {
    ($sender:expr, $($arg:tt)*) => {{
        let mut line = heapless::String::<512>::new();
        let _ = core::write!(&mut line, $($arg)*);
        let _ = line.push_str("\r\n");
        $sender.write_packet(line.as_bytes()).await
    }};
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // MAX16169 CLR is active-low. Keep GPIO8 released as an input and let the
    // board's 10k pull-up hold the power latch without an output-low glitch.
    let power_kill_n = Input::new(
        peripherals.GPIO8,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Establish safe peripheral states before starting either USB or I2C.
    let mut led_enable = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    let _lcd_backlight = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());

    let charger_interrupt = Input::new(peripherals.GPIO0, InputConfig::default());
    let charger_status = Input::new(peripherals.GPIO1, InputConfig::default());
    let battery_alert = Input::new(peripherals.GPIO2, InputConfig::default());
    let power_interrupt = Input::new(peripherals.GPIO38, InputConfig::default());
    let charger_power_good = Input::new(peripherals.GPIO40, InputConfig::default());
    let usb_vbus_sense = Input::new(peripherals.GPIO35, InputConfig::default());
    let boot_button = Input::new(
        peripherals.GPIO61,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Start both reset nets high. The IMU gets one documented hardware reset
    // pulse before its read-only connection probe; GNSS remains untouched.
    let mut imu_reset_n = Output::new(peripherals.GPIO9, Level::High, OutputConfig::default());
    let gnss_reset_n = Input::new(
        peripherals.GPIO44,
        InputConfig::default().with_pull(Pull::Up),
    );
    let imu_data_ready = Input::new(peripherals.GPIO14, InputConfig::default());

    let mut imu_chip_select = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let mut imu_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .expect("SPI2 initialization failed")
    .with_sck(peripherals.GPIO12)
    .with_miso(peripherals.GPIO13)
    .with_mosi(peripherals.GPIO11);

    let mut gnss_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(1)),
    )
    .expect("UART1 initialization failed")
    .with_tx(peripherals.GPIO47)
    .with_rx(peripherals.GPIO46)
    .into_async();
    let mut gnss_uart_2 = Uart::new(
        peripherals.UART2,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(1)),
    )
    .expect("UART2 initialization failed")
    .with_tx(peripherals.GPIO49)
    .with_rx(peripherals.GPIO48)
    .into_async();

    let mut power_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .expect("I2C0 initialization failed")
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO6);

    // J2 is wired to the ESP32-S31's native 4-bit SDHOST pin set. The slot
    // has no connected mechanical card-detect switch, so presence is proved
    // by protocol initialization and a read of logical block zero.
    let sd_controller =
        SdHostController::new(peripherals.SDHOST, SdConfig::default()).expect("SDHOST init failed");
    let sd_slot = sd_controller
        .slot::<0>(SlotConfig::default())
        .expect("SDHOST slot 0 unavailable")
        .with_clk(peripherals.GPIO24)
        .with_cmd(peripherals.GPIO25)
        .with_data0(peripherals.GPIO20)
        .with_data1(peripherals.GPIO21)
        .with_data2(peripherals.GPIO22)
        .with_data3(peripherals.GPIO23)
        .into_async();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // J6 is wired to the ESP32-S31 dedicated high-speed USB controller.
    let usb = Usb::new_hs(peripherals.USB_HS);
    let mut endpoint_buffer = [0_u8; 1024];
    let driver = Driver::new(usb, &mut endpoint_buffer, UsbDriverConfig::default());

    let mut usb_config = embassy_usb::Config::new(0x303A, 0x4001);
    usb_config.max_packet_size_0 = 64;
    usb_config.manufacturer = Some("AEVIA");
    usb_config.product = Some("V2 Mini Bring-up Console");
    usb_config.serial_number = Some("V2MINI0001");

    let mut config_descriptor = [0_u8; 256];
    let mut bos_descriptor = [0_u8; 256];
    let mut control_buffer = [0_u8; 64];
    let mut cdc_state = State::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buffer,
    );
    let serial = CdcAcmClass::new(&mut builder, &mut cdc_state, 512);
    let (mut sender, mut receiver) = serial.split();
    let mut usb_device = builder.build();

    let usb_task = usb_device.run();

    let report_task = async {
        let mut sd_init_error = None;
        let mut sd_read_error = None;
        let mut sd_capacity_bytes = 0_u64;
        let mut sd_lba0_prefix = [0_u8; 16];
        let mut sd_lba0_signature = [0_u8; 2];

        match SdBlockDevice::<Card, _, _, 512>::new_sd_card(sd_slot, 10_000_000, Delay).await {
            Ok(mut card) => {
                sd_capacity_bytes = BlockDeviceDriver::<512>::size(&mut card).await.unwrap_or(0);
                let mut lba0 = [Aligned::<A4, _>([0_u8; 512])];
                match BlockDeviceDriver::<512>::read(&mut card, 0, &mut lba0).await {
                    Ok(()) => {
                        sd_lba0_prefix.copy_from_slice(&lba0[0][..16]);
                        sd_lba0_signature.copy_from_slice(&lba0[0][510..512]);
                    }
                    Err(error) => sd_read_error = Some(error),
                }
            }
            Err(error) => sd_init_error = Some(error),
        }

        let mut pass = 0_u32;
        let mut imu_reset_performed = false;
        let mut gnss_uart_1_seen = false;
        let mut gnss_uart_2_seen = false;
        let mut gnss_gga_requested = false;

        loop {
            sender.wait_connection().await;

            'connected: loop {
                macro_rules! report_line {
                    ($($arg:tt)*) => {
                        if usb_line!(sender, $($arg)*).is_err() {
                            break 'connected;
                        }
                    };
                }

                pass = pass.wrapping_add(1);
                if !imu_reset_performed {
                    imu_reset_n.set_low();
                    Timer::after_millis(2).await;
                    imu_reset_n.set_high();
                    Timer::after_millis(10).await;
                    imu_reset_performed = true;
                }
                report_line!("Hello, world!");
                report_line!(
                    "AEVIA V2 Mini peripheral bring-up v{}",
                    env!("CARGO_PKG_VERSION")
                );
                report_line!("--- bring-up pass {} ---", pass);
                report_line!(
                    "GPIO PWR_KILL_N={} PWR_INT_N={} CHG_PG={} CHG_INT={} CHG_STAT={} BAT_ALRT={} VBUS={} BOOT={}",
                    level(power_kill_n.is_high()),
                    level(power_interrupt.is_high()),
                    level(charger_power_good.is_high()),
                    level(charger_interrupt.is_high()),
                    level(charger_status.is_high()),
                    level(battery_alert.is_high()),
                    level(usb_vbus_sense.is_high()),
                    level(boot_button.is_high()),
                );

                if let Some(error) = sd_init_error {
                    report_line!(
                        "[MISS] microSD J2 native 4-bit init error={:?} (no card-detect pin)",
                        error,
                    );
                } else if let Some(error) = sd_read_error {
                    report_line!(
                        "[FAIL] microSD J2 initialized capacity={} bytes, LBA0 read error={:?}",
                        sd_capacity_bytes,
                        error,
                    );
                } else {
                    report_line!(
                        "[PASS] microSD J2 native 4-bit read-only: capacity={} bytes LBA0[0..16]={:02X?} signature={:02X?}",
                        sd_capacity_bytes,
                        sd_lba0_prefix,
                        sd_lba0_signature,
                    );
                }

                let bq_ok = match read_bq2562x(&mut power_i2c) {
                    Ok(snapshot) => {
                        let expected = snapshot.is_supported_charger();
                        report_line!(
                            "[{}] {} @ 0x6B part=0x{:02X} pn={} rev={} status={:02X}/{:02X} fault={:02X}",
                            if expected { "PASS" } else { "FAIL" },
                            snapshot.part_name(),
                            snapshot.part_information,
                            snapshot.part_number(),
                            snapshot.device_revision(),
                            snapshot.charger_status_0,
                            snapshot.charger_status_1,
                            snapshot.fault_status_0,
                        );
                        report_line!(
                            "       cfg (read-only): ICHG={:04X} VREG={:04X} VSYSMIN={:04X} CTRL0={:02X} CTRL3={:02X}; charge={} vbus={}",
                            snapshot.charge_current_limit,
                            snapshot.charge_voltage_limit,
                            snapshot.minimum_system_voltage,
                            snapshot.charger_control_0,
                            snapshot.charger_control_3,
                            charge_state(snapshot.charge_state()),
                            vbus_state(snapshot.vbus_state()),
                        );
                        expected
                    }
                    Err(error) => {
                        report_line!("[MISS] BQ25622 @ 0x6B error={:?}", error);
                        false
                    }
                };

                // Retry at three in-spec clocks. Each pass performs Murata's
                // documented software reset, then accounts for the pipelined
                // response by issuing the component-ID request twice.
                let imu_clock_khz = match pass % 3 {
                    1 => 100,
                    2 => 1_000,
                    _ => 10_000,
                };
                let imu_config_ok = imu_spi
                    .apply_config(
                        &SpiConfig::default()
                            .with_frequency(Rate::from_khz(imu_clock_khz))
                            .with_mode(Mode::_0),
                    )
                    .is_ok();
                let mut imu_response = sch16t_request_bytes(SCH16T_SOFT_RESET);
                imu_chip_select.set_low();
                let soft_reset_ok =
                    embedded_hal::spi::SpiBus::transfer_in_place(&mut imu_spi, &mut imu_response)
                        .is_ok();
                imu_chip_select.set_high();
                Timer::after_millis(32).await;

                let mut imu_response = sch16t_request_bytes(SCH16T_READ_COMPONENT_ID);
                imu_chip_select.set_low();
                let first_spi_ok =
                    embedded_hal::spi::SpiBus::transfer_in_place(&mut imu_spi, &mut imu_response)
                        .is_ok();
                imu_chip_select.set_high();
                Timer::after_micros(1).await;

                imu_response = sch16t_request_bytes(SCH16T_READ_COMPONENT_ID);
                imu_chip_select.set_low();
                let second_spi_ok =
                    embedded_hal::spi::SpiBus::transfer_in_place(&mut imu_spi, &mut imu_response)
                        .is_ok();
                imu_chip_select.set_high();

                let imu_frame = sch16t_frame(imu_response);
                let imu_ok = imu_config_ok
                    && soft_reset_ok
                    && first_spi_ok
                    && second_spi_ok
                    && sch16t_frame_has_valid_crc(imu_frame);
                if imu_ok {
                    report_line!(
                        "[PASS] SCH16T SafeSPI component_id=0x{:04X} frame=0x{:012X} crc=ok clock={}kHz RESET_N={} DRDY={}",
                        sch16t_component_id(imu_frame),
                        imu_frame,
                        imu_clock_khz,
                        level(imu_reset_n.is_set_high()),
                        level(imu_data_ready.is_high()),
                    );
                } else {
                    report_line!(
                        "[MISS] SCH16T SafeSPI frame=0x{:012X} config={} soft_reset={} reads={}/{} crc=bad clock={}kHz RESET_N={} DRDY={}",
                        imu_frame,
                        verified(imu_config_ok),
                        verified(soft_reset_ok),
                        verified(first_spi_ok),
                        verified(second_spi_ok),
                        imu_clock_khz,
                        level(imu_reset_n.is_set_high()),
                        level(imu_data_ready.is_high()),
                    );
                }

                // VERSIONA is read-only and returns receiver identity/version
                // without requiring satellite reception or an antenna.
                let gnss_write_ok =
                    embedded_io_async::Write::write_all(&mut gnss_uart, b"VERSIONA\r\n")
                        .await
                        .is_ok()
                        && embedded_io_async::Write::flush(&mut gnss_uart)
                            .await
                            .is_ok();
                let mut gnss_response = [0_u8; 512];
                let mut gnss_length = 0_usize;
                for _ in 0..10 {
                    if gnss_length == gnss_response.len()
                        || is_um980_connection_response(&gnss_response[..gnss_length])
                    {
                        break;
                    }

                    match with_timeout(
                        Duration::from_millis(100),
                        embedded_io_async::Read::read(
                            &mut gnss_uart,
                            &mut gnss_response[gnss_length..],
                        ),
                    )
                    .await
                    {
                        Ok(Ok(length)) => gnss_length += length,
                        Ok(Err(_)) | Err(_) => {}
                    }
                }
                gnss_uart_1_seen |=
                    gnss_write_ok && is_um980_connection_response(&gnss_response[..gnss_length]);
                let gnss_uart_1_ok = gnss_uart_1_seen;
                if gnss_uart_1_ok {
                    report_line!(
                        "[PASS] UM980 UART1 115200 response ({} bytes; type={} RESET_N={})",
                        gnss_length,
                        if is_um980_version_reply(&gnss_response[..gnss_length]) {
                            "identity"
                        } else {
                            "NMEA/command"
                        },
                        level(gnss_reset_n.is_high()),
                    );
                } else {
                    report_line!(
                        "[MISS] UM980 UART1 115200 VERSIONA response (tx={} rx_bytes={} RESET_N={} raw={:02X?})",
                        verified(gnss_write_ok),
                        gnss_length,
                        level(gnss_reset_n.is_high()),
                        &gnss_response[..gnss_length],
                    );
                }

                // Enable a volatile 1 Hz GGA stream on the port that already
                // proved electrically connected. No SAVECONFIG is issued, so
                // this diagnostic does not alter the receiver's saved setup.
                let gnss_write_2_ok = if gnss_gga_requested {
                    true
                } else {
                    let requested =
                        embedded_io_async::Write::write_all(&mut gnss_uart_2, b"GPGGA 1\r\n")
                            .await
                            .is_ok()
                            && embedded_io_async::Write::flush(&mut gnss_uart_2)
                                .await
                                .is_ok();
                    gnss_gga_requested = requested;
                    requested
                };
                let mut gnss_response_2 = [0_u8; 512];
                let mut gnss_length_2 = 0_usize;
                // The ESP UART can yield only a byte or two per read. Give a
                // full 1 Hz NMEA interval enough reads to assemble one line.
                for _ in 0..160 {
                    if gnss_length_2 == gnss_response_2.len()
                        || parse_um980_gga(&gnss_response_2[..gnss_length_2]).is_some()
                    {
                        break;
                    }

                    match with_timeout(
                        Duration::from_millis(20),
                        embedded_io_async::Read::read(
                            &mut gnss_uart_2,
                            &mut gnss_response_2[gnss_length_2..],
                        ),
                    )
                    .await
                    {
                        Ok(Ok(length)) => gnss_length_2 += length,
                        Ok(Err(_)) | Err(_) => {}
                    }
                }
                gnss_uart_2_seen |= gnss_write_2_ok
                    && is_um980_connection_response(&gnss_response_2[..gnss_length_2]);
                let gnss_uart_2_ok = gnss_uart_2_seen;
                let gga = parse_um980_gga(&gnss_response_2[..gnss_length_2]);
                let gnss_fix_ok = gga
                    .map(|gga| gga.fix_quality != 0 && gga.coordinates_present)
                    .unwrap_or(false);
                if let Some(gga) = gga {
                    report_line!(
                        "[{}] UM980 UART2 GGA fix_quality={} satellites={} coordinates={} RESET_N={}",
                        if gnss_fix_ok { "FIX" } else { "NO FIX" },
                        gga.fix_quality,
                        gga.satellites,
                        verified(gga.coordinates_present),
                        level(gnss_reset_n.is_high()),
                    );
                } else if gnss_uart_2_ok {
                    report_line!(
                        "[LINK] UM980 UART2 115200 response, but no complete GGA yet ({} bytes; type={} RESET_N={} raw={:02X?})",
                        gnss_length_2,
                        if is_um980_version_reply(&gnss_response_2[..gnss_length_2]) {
                            "identity"
                        } else {
                            "NMEA/command"
                        },
                        level(gnss_reset_n.is_high()),
                        &gnss_response_2[..gnss_length_2.min(96)],
                    );
                } else {
                    report_line!(
                        "[MISS] UM980 UART2 115200 GPGGA response (tx={} rx_bytes={} RESET_N={} raw_prefix={:02X?})",
                        verified(gnss_write_2_ok),
                        gnss_length_2,
                        level(gnss_reset_n.is_high()),
                        &gnss_response_2[..gnss_length_2.min(96)],
                    );
                }
                let gnss_ok = gnss_uart_1_ok || gnss_uart_2_ok;

                let gauge_ok = match read_max17048(&mut power_i2c) {
                    Ok(snapshot) => {
                        let soc = snapshot.state_of_charge_hundredths();
                        let expected = snapshot.has_expected_version();
                        report_line!(
                            "[{}] MAX17048 @ 0x36 version={:04X} cell={}mV soc={}.{:02}% status={:04X}",
                            if expected { "PASS" } else { "WARN" },
                            snapshot.version,
                            snapshot.cell_millivolts(),
                            soc / 100,
                            soc % 100,
                            snapshot.status,
                        );
                        expected
                    }
                    Err(_) => {
                        report_line!(
                            "[NACK] MAX17048 @ 0x36 (acceptable with no cell; measure BAT)"
                        );
                        false
                    }
                };

                let buttons_ok = match read_tca9536a(&mut power_i2c) {
                    Ok(snapshot) => {
                        let expected = snapshot.is_reset_and_idle();
                        report_line!(
                            "[{}] TCA9536A @ 0x40 inputs={:02X} config={:02X} (buttons=P3..P0, active-low)",
                            if expected { "PASS" } else { "WARN" },
                            snapshot.inputs,
                            snapshot.configuration,
                        );
                        expected
                    }
                    Err(_) => {
                        report_line!("[MISS] TCA9536A @ 0x40 did not answer");
                        false
                    }
                };

                // The active LED test reproducibly reset the whole board as
                // soon as LP5813 chip_en was asserted, before any current/PWM
                // write. Leave the boost disabled and keep the console alive.
                led_enable.set_high();
                Timer::after_millis(2).await;
                let led_snapshot = read_lp5813a_reset_state(&mut power_i2c);
                led_enable.set_low();
                let led_ok = match led_snapshot {
                    Ok(snapshot) => {
                        report_line!(
                            "[FAIL] LP5813A responds @ 0x50, but chip_en caused board reset; active LED test disabled (safe regs={:02X}/{:02X}/{:02X}/{:02X})",
                            snapshot.chip_enable,
                            snapshot.device_config_0,
                            snapshot.device_config_1,
                            snapshot.device_config_2,
                        );
                        false
                    }
                    Err(_) => {
                        report_line!("[MISS] LP5813A @ 0x50 did not answer while LED_EN was high");
                        false
                    }
                };

                report_line!(
                    "SUMMARY charger={} imu={} gnss_link={} gnss_fix={} gauge={} buttons={} led_driver={} microsd={} (next pass in 5s)",
                    verified(bq_ok),
                    verified(imu_ok),
                    verified(gnss_ok),
                    verified(gnss_fix_ok),
                    verified(gauge_ok),
                    verified(buttons_ok),
                    verified(led_ok),
                    verified(sd_init_error.is_none() && sd_read_error.is_none()),
                );
                report_line!("Send BOOTLOADER to reflash without pressing board buttons.");
                report_line!("");
                Timer::after_secs(5).await;
            }
        }
    };

    let command_task = async {
        let mut packet = [0_u8; 512];

        loop {
            receiver.wait_connection().await;
            loop {
                match receiver.read_packet(&mut packet).await {
                    Ok(length) if contains_bootloader_command(&packet[..length]) => {
                        // LP_SYS.FORCE_DOWNLOAD_BOOT is sampled by ROM after the
                        // software reset. ROM then exposes J6 as its flash port.
                        esp_hal::peripherals::LP_SYS::regs()
                            .sys_ctrl()
                            .modify(|_, writer| writer.force_download_boot().set_bit());
                        esp_hal::system::software_reset();
                    }
                    Ok(_) => {}
                    Err(_) => break,
                }
            }
        }
    };

    join3(usb_task, report_task, command_task).await;
}

fn contains_bootloader_command(packet: &[u8]) -> bool {
    const COMMAND: &[u8] = b"BOOTLOADER";
    packet
        .windows(COMMAND.len())
        .any(|window| window == COMMAND)
}

const fn level(high: bool) -> &'static str {
    if high { "H" } else { "L" }
}

const fn verified(value: bool) -> &'static str {
    if value { "yes" } else { "no" }
}

const fn charge_state(value: u8) -> &'static str {
    match value {
        0 => "idle/done",
        1 => "CC",
        2 => "CV",
        3 => "top-off",
        _ => "?",
    }
}

const fn vbus_state(value: u8) -> &'static str {
    match value {
        0 => "absent",
        4 => "present",
        _ => "reserved",
    }
}
