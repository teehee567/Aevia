//! V2 Mini assignments verified against U2 pads in the PCB, using proven drivers.
use embassy_executor::Spawner;
use esp_hal::{
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    uart::{Config as UartConfig, RxConfig, Uart},
};

#[allow(non_snake_case)]
pub struct Hardware {
    pub GPIO20: esp_hal::peripherals::GPIO20<'static>,
    pub GPIO21: esp_hal::peripherals::GPIO21<'static>,
    pub GPIO22: esp_hal::peripherals::GPIO22<'static>,
    pub GPIO23: esp_hal::peripherals::GPIO23<'static>,
    pub GPIO24: esp_hal::peripherals::GPIO24<'static>,
    pub GPIO25: esp_hal::peripherals::GPIO25<'static>,
    pub SDHOST: esp_hal::peripherals::SDHOST<'static>,
    pub GPIO4: esp_hal::peripherals::GPIO4<'static>,
    pub GPIO15: esp_hal::peripherals::GPIO15<'static>,
    pub GPIO16: esp_hal::peripherals::GPIO16<'static>,
    pub GPIO17: esp_hal::peripherals::GPIO17<'static>,
    pub GPIO18: esp_hal::peripherals::GPIO18<'static>,
    pub GPIO19: esp_hal::peripherals::GPIO19<'static>,
    pub SPI3: esp_hal::peripherals::SPI3<'static>,
    pub GPIO8: esp_hal::peripherals::GPIO8<'static>,
    pub GPIO47: esp_hal::peripherals::GPIO47<'static>,
    pub GPIO46: esp_hal::peripherals::GPIO46<'static>,
    pub GPIO49: esp_hal::peripherals::GPIO49<'static>,
    pub GPIO48: esp_hal::peripherals::GPIO48<'static>,
    pub GPIO12: esp_hal::peripherals::GPIO12<'static>,
    pub GPIO13: esp_hal::peripherals::GPIO13<'static>,
    pub GPIO11: esp_hal::peripherals::GPIO11<'static>,
    pub GPIO10: esp_hal::peripherals::GPIO10<'static>,
    pub GPIO9: esp_hal::peripherals::GPIO9<'static>,
    pub GPIO14: esp_hal::peripherals::GPIO14<'static>,
    pub GPIO6: esp_hal::peripherals::GPIO6<'static>,
    pub GPIO7: esp_hal::peripherals::GPIO7<'static>,
    pub GPIO39: esp_hal::peripherals::GPIO39<'static>,
    pub GPIO40: esp_hal::peripherals::GPIO40<'static>,
    pub GPIO42: esp_hal::peripherals::GPIO42<'static>,
    pub UART1: esp_hal::peripherals::UART1<'static>,
    pub UART2: esp_hal::peripherals::UART2<'static>,
    pub SPI2: esp_hal::peripherals::SPI2<'static>,
    pub I2C0: esp_hal::peripherals::I2C0<'static>,
    pub FROM_CPU_INTR2: esp_hal::peripherals::FROM_CPU_INTR2<'static>,
    pub FROM_CPU_INTR3: esp_hal::peripherals::FROM_CPU_INTR3<'static>,
    pub IO_MUX: esp_hal::peripherals::IO_MUX<'static>,
}

pub fn start(
    spawner: Spawner,
    mut p: Hardware,
) -> esp_hal::sdmmc::Slot<'static, 0, esp_hal::Blocking> {
    // Hold the open-drain power kill inactive for the lifetime of the application.
    let _kill = Output::new(
        p.GPIO8,
        Level::High,
        OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::Up),
    );
    let gnss_command = Uart::new(
        p.UART1,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(1)),
    )
    .unwrap()
    .with_tx(p.GPIO47)
    .with_rx(p.GPIO46);
    let gnss_data = Uart::new(
        p.UART2,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(64)),
    )
    .unwrap()
    .with_tx(p.GPIO49)
    .with_rx(p.GPIO48);
    let imu_spi = Spi::new(
        p.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(8))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO12)
    .with_miso(p.GPIO13)
    .with_mosi(p.GPIO11);
    let imu = crate::drivers::sch16t::Imu::new(
        imu_spi,
        Output::new(p.GPIO10, Level::High, OutputConfig::default()),
        Output::new(p.GPIO9, Level::High, OutputConfig::default()),
        Input::new(p.GPIO14, InputConfig::default()),
    );
    let _pps = Input::new(p.GPIO42, InputConfig::default());
    crate::capture::init(42, 14);
    spawner.spawn(crate::timing::maintenance().unwrap());
    let bus = I2c::new(
        p.I2C0,
        I2cConfig::default()
            .with_frequency(Rate::from_khz(100))
            .with_timeout(BusTimeout::BusCycles(100))
            .with_software_timeout(SoftwareTimeout::Transaction(
                esp_hal::time::Duration::from_millis(50),
            )),
    )
    .unwrap()
    .with_scl(p.GPIO6)
    .with_sda(p.GPIO7)
    .into_async();
    let led = Output::new(p.GPIO39, Level::Low, OutputConfig::default());
    let power_good = Input::new(p.GPIO40, InputConfig::default().with_pull(Pull::Up));
    let sclk = if crate::configuration::DISPLAY_FITTED {
        crate::drivers::st7789::check_sclk(p.GPIO16.reborrow())
    } else {
        0
    };
    let lcd_spi = Spi::new(
        p.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sio0(p.GPIO17);
    let lcd_spi = if sclk == 26 {
        lcd_spi.with_sck(p.GPIO16)
    } else {
        lcd_spi
    };
    let lcd = crate::drivers::st7789::Lcd::new(
        lcd_spi,
        Output::new(p.GPIO18, Level::High, OutputConfig::default()),
        Output::new(p.GPIO19, Level::Low, OutputConfig::default()),
        Output::new(p.GPIO15, Level::High, OutputConfig::default()),
    );
    let backlight = Output::new(p.GPIO4, Level::Low, OutputConfig::default());
    // GPIO must wake the priority-2 IMU executor even while priority-1 GNSS
    // parsing is running; neither interrupt task performs floating-point work.
    esp_hal::gpio::Io::new(p.IO_MUX)
        .set_interrupt_priority(esp_hal::interrupt::Priority::Priority3);
    static GNSS_EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::InterruptExecutor<3>> =
        static_cell::StaticCell::new();
    GNSS_EXECUTOR
        .init(esp_rtos::embassy::InterruptExecutor::new(p.FROM_CPU_INTR3))
        .start(esp_hal::interrupt::Priority::Priority1)
        .spawn(crate::app::gnss_acquire(gnss_command, gnss_data).unwrap());
    spawner.spawn(crate::app::gnss().unwrap());
    static ACQUISITION_EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::InterruptExecutor<2>> =
        static_cell::StaticCell::new();
    let acquisition = ACQUISITION_EXECUTOR
        .init(esp_rtos::embassy::InterruptExecutor::new(p.FROM_CPU_INTR2))
        .start(esp_hal::interrupt::Priority::Priority2);
    acquisition.spawn(crate::app::imu_acquire(imu).unwrap());
    spawner.spawn(crate::app::imu().unwrap());
    spawner.spawn(crate::tasks::power::task(bus, led, power_good).unwrap());
    spawner.spawn(crate::tasks::display::run(lcd, backlight, sclk).unwrap());
    static SD: static_cell::StaticCell<esp_hal::sdmmc::SdHostController<'static>> =
        static_cell::StaticCell::new();
    let sd = SD.init(esp_hal::sdmmc::SdHostController::new(p.SDHOST, Default::default()).unwrap());
    sd.slot::<0>(Default::default())
        .unwrap()
        .with_clk(p.GPIO24)
        .with_cmd(p.GPIO25)
        .with_data0(p.GPIO20)
        .with_data1(p.GPIO21)
        .with_data2(p.GPIO22)
        .with_data3(p.GPIO23)
}
