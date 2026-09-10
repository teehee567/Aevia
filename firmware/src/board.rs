//! Frozen V2 Mini wiring and bus configuration. This is the only GPIO assignment site.
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    peripherals::{CPU_CTRL, FROM_CPU_INTR0, FROM_CPU_INTR1, PSRAM, TIMG0, USB_HS},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    uart::{Config as UartConfig, RxConfig, Uart},
};

pub struct Board {
    pub power_bus: I2c<'static, Async>,
    pub led_enable: Output<'static>,
    pub charger_power_good_n: Input<'static>,
    pub power_interrupt_n: Input<'static>,
    pub power_kill_n: Output<'static>,
    pub gnss_command: Uart<'static, Async>,
    pub gnss_data: Uart<'static, Async>,
    pub gnss_reset_n: Input<'static>,
    pub imu: crate::drivers::sch16t::Imu<'static>,
    pub lcd: crate::drivers::st7789::Lcd,
    pub lcd_te: Input<'static>,
    pub lcd_sclk_probe: u32,
    pub backlight: Output<'static>,
    pub usb: USB_HS<'static>,
    pub psram: PSRAM<'static>,
    pub cpu: CPU_CTRL<'static>,
    pub timer: TIMG0<'static>,
    pub core0_interrupt: FROM_CPU_INTR0<'static>,
    pub core1_interrupt: FROM_CPU_INTR1<'static>,
}

pub fn initialize() -> Board {
    let mut peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let power_kill_n = Output::new(
        peripherals.GPIO8,
        Level::High,
        OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::Up),
    );
    let power_interrupt_n = Input::new(
        peripherals.GPIO38,
        InputConfig::default().with_pull(Pull::Up),
    );
    let led_enable = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    let charger_power_good_n = Input::new(
        peripherals.GPIO40,
        InputConfig::default().with_pull(Pull::Up),
    );
    let lcd_sclk_probe = crate::drivers::st7789::check_sclk(peripherals.GPIO16.reborrow());
    let lcd_spi = Spi::new(
        peripherals.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .expect("LCD SPI")
    .with_sio0(peripherals.GPIO17);
    let lcd_spi = if lcd_sclk_probe == 26 {
        lcd_spi.with_sck(peripherals.GPIO16)
    } else {
        lcd_spi
    };
    let lcd = crate::drivers::st7789::Lcd::new(
        lcd_spi,
        Output::new(peripherals.GPIO18, Level::High, OutputConfig::default()),
        Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default()),
        Output::new(peripherals.GPIO15, Level::High, OutputConfig::default()),
    );
    let lcd_te = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Down),
    );
    let gnss_reset_n = Input::new(
        peripherals.GPIO44,
        InputConfig::default().with_pull(Pull::Up),
    );
    let power_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default()
            .with_frequency(Rate::from_khz(100))
            .with_timeout(BusTimeout::BusCycles(100))
            .with_software_timeout(SoftwareTimeout::Transaction(
                esp_hal::time::Duration::from_millis(10),
            )),
    )
    .expect("power I2C")
    .with_scl(peripherals.GPIO6)
    .with_sda(peripherals.GPIO7)
    .into_async();

    // Schematic pin assignment. See docs/gnss-usb-bridge.md for the assembled
    // board's unresolved COM1 RX / COM2 TX connectivity findings.
    let gnss_command_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(1)),
    )
    .expect("GNSS COM1 UART")
    .with_tx(peripherals.GPIO47)
    .with_rx(peripherals.GPIO46)
    .into_async();
    let gnss_data_uart = Uart::new(
        peripherals.UART2,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(64)),
    )
    .expect("GNSS COM2 UART")
    .with_tx(peripherals.GPIO49)
    .with_rx(peripherals.GPIO48)
    .into_async();

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .expect("IMU SPI")
    .with_sck(peripherals.GPIO12)
    .with_miso(peripherals.GPIO13)
    .with_mosi(peripherals.GPIO11);
    let sensor = crate::drivers::sch16t::Imu::new(
        spi,
        Output::new(peripherals.GPIO10, Level::High, OutputConfig::default()),
        Output::new(peripherals.GPIO9, Level::High, OutputConfig::default()),
        Input::new(peripherals.GPIO14, InputConfig::default()),
    );

    Board {
        power_bus: power_i2c,
        led_enable,
        charger_power_good_n,
        power_interrupt_n,
        power_kill_n,
        gnss_command: gnss_command_uart,
        gnss_data: gnss_data_uart,
        gnss_reset_n,
        imu: sensor,
        lcd,
        lcd_te,
        lcd_sclk_probe,
        backlight: Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default()),
        usb: peripherals.USB_HS,
        psram: peripherals.PSRAM,
        cpu: peripherals.CPU_CTRL,
        timer: peripherals.TIMG0,
        core0_interrupt: peripherals.FROM_CPU_INTR0,
        core1_interrupt: peripherals.FROM_CPU_INTR1,
    }
}
