//! The single composition point for fitted V2 Mini resources. Wiring comes
//! from HAL's PCB-derived constants; no task or runtime hands off numbered pins.
use super::{display::Display, imu::Imu, platform};
use aevia_device_hal::{
    board as wiring,
    esp32s31::{self as native, GnssUartSettings},
};
use esp_hal::{
    Async, Blocking,
    gpio::{AnyPin, DriveMode, Input, InputConfig, Io, Level, Output, OutputConfig, Pin, Pull},
    i2c::master::I2c,
    interrupt::{self, Priority},
    peripherals::*,
    uart::Uart,
};

pub(crate) type PowerBus = I2c<'static, Blocking>;
pub(crate) type RecordingSlot = esp_hal::sdmmc::Slot<'static, 0, Blocking>;
static SD_CONTROLLER: static_cell::StaticCell<esp_hal::sdmmc::SdHostController<'static>> =
    static_cell::StaticCell::new();
// Connected evaluation board: battery fitted, no panel or buttons. Absent
// accessories need no periodic I/O.
pub(crate) use super::configuration::{BATTERY_FITTED, BUTTONS_FITTED, DISPLAY_FITTED};
pub(crate) const ACQUISITION_PRIORITY: Priority = Priority::Priority5;
pub(crate) const GNSS_PRIORITY: Priority = Priority::Priority3;
const SENSOR_INTERRUPT_PRIORITY: Priority = Priority::Priority6;

pub(crate) struct Board {
    pub timers: TIMG0<'static>,
    pub scheduler_interrupt: FROM_CPU_INTR0<'static>,
    pub acquisition_interrupt: FROM_CPU_INTR2<'static>,
    pub gnss_interrupt: FROM_CPU_INTR3<'static>,
    pub flash: FLASH<'static>,
    pub usb: USB_HS<'static>,
    pub power_bus: PowerBus,
    pub application: ApplicationBuilder,
}

struct Pins([Option<AnyPin<'static>>; 62]);
impl Pins {
    fn take(&mut self, number: u8) -> AnyPin<'static> {
        self.0[usize::from(number)]
            .take()
            .expect("board pin already assigned")
    }
    fn output(&mut self, number: u8, level: Level) -> Output<'static> {
        Output::new(self.take(number), level, OutputConfig::default())
    }
}

pub(crate) struct ApplicationBuilder {
    pins: Pins,
    uart1: UART1<'static>,
    uart2: UART2<'static>,
    spi2: SPI2<'static>,
    spi3: SPI3<'static>,
    sdhost: SDHOST<'static>,
    psram: PSRAM<'static>,
    cpu: CPU_CTRL<'static>,
    worker_interrupt: FROM_CPU_INTR1<'static>,
    io_mux: IO_MUX<'static>,
}

pub(crate) struct SensorResources {
    pub imu: Imu<'static>,
    pub command: Uart<'static, Blocking>,
    pub data: Uart<'static, Blocking>,
    pub gnss_reset: Output<'static>,
    pub pps: Input<'static>,
}

pub(crate) struct MaintenanceResources {
    pub led_enable: Output<'static>,
    pub power_hold: Output<'static>,
}

pub(crate) struct Application {
    pub sensors: SensorResources,
    pub display: Display,
    pub maintenance: MaintenanceResources,
    pub memory: platform::EstimatorMemory,
    pub cpu: CPU_CTRL<'static>,
    pub worker_interrupt: FROM_CPU_INTR1<'static>,
    pub recording_slot: RecordingSlot,
}

impl Board {
    pub(crate) fn new(p: Peripherals) -> Self {
        // Inventory only: erasing pin types performs no GPIO configuration.
        // Each connection below takes its owner using the HAL wiring constant.
        let mut pins = Pins([
            Some(p.GPIO0.degrade()),
            Some(p.GPIO1.degrade()),
            Some(p.GPIO2.degrade()),
            Some(p.GPIO3.degrade()),
            Some(p.GPIO4.degrade()),
            Some(p.GPIO5.degrade()),
            Some(p.GPIO6.degrade()),
            Some(p.GPIO7.degrade()),
            Some(p.GPIO8.degrade()),
            Some(p.GPIO9.degrade()),
            Some(p.GPIO10.degrade()),
            Some(p.GPIO11.degrade()),
            Some(p.GPIO12.degrade()),
            Some(p.GPIO13.degrade()),
            Some(p.GPIO14.degrade()),
            Some(p.GPIO15.degrade()),
            Some(p.GPIO16.degrade()),
            Some(p.GPIO17.degrade()),
            Some(p.GPIO18.degrade()),
            Some(p.GPIO19.degrade()),
            Some(p.GPIO20.degrade()),
            Some(p.GPIO21.degrade()),
            Some(p.GPIO22.degrade()),
            Some(p.GPIO23.degrade()),
            Some(p.GPIO24.degrade()),
            Some(p.GPIO25.degrade()),
            Some(p.GPIO26.degrade()),
            Some(p.GPIO27.degrade()),
            Some(p.GPIO28.degrade()),
            None, // No GPIO29 owner in the S31 SDK.
            Some(p.GPIO30.degrade()),
            Some(p.GPIO31.degrade()),
            Some(p.GPIO32.degrade()),
            Some(p.GPIO33.degrade()),
            Some(p.GPIO34.degrade()),
            Some(p.GPIO35.degrade()),
            Some(p.GPIO36.degrade()),
            Some(p.GPIO37.degrade()),
            Some(p.GPIO38.degrade()),
            Some(p.GPIO39.degrade()),
            Some(p.GPIO40.degrade()),
            None, // No GPIO41 owner in the S31 SDK.
            Some(p.GPIO42.degrade()),
            Some(p.GPIO43.degrade()),
            Some(p.GPIO44.degrade()),
            Some(p.GPIO45.degrade()),
            Some(p.GPIO46.degrade()),
            Some(p.GPIO47.degrade()),
            Some(p.GPIO48.degrade()),
            Some(p.GPIO49.degrade()),
            Some(p.GPIO50.degrade()),
            Some(p.GPIO51.degrade()),
            Some(p.GPIO52.degrade()),
            Some(p.GPIO53.degrade()),
            Some(p.GPIO54.degrade()),
            Some(p.GPIO55.degrade()),
            Some(p.GPIO56.degrade()),
            Some(p.GPIO57.degrade()),
            Some(p.GPIO58.degrade()),
            Some(p.GPIO59.degrade()),
            Some(p.GPIO60.degrade()),
            Some(p.GPIO61.degrade()),
        ]);
        let power_bus = native::init_power_i2c(
            p.I2C0,
            pins.take(wiring::POWER_I2C.scl)
                .try_into()
                .expect("HAL power SCL wiring"),
            pins.take(wiring::POWER_I2C.sda)
                .try_into()
                .expect("HAL power SDA wiring"),
        )
        .expect("power I2C");
        Self {
            timers: p.TIMG0,
            scheduler_interrupt: p.FROM_CPU_INTR0,
            acquisition_interrupt: p.FROM_CPU_INTR2,
            gnss_interrupt: p.FROM_CPU_INTR3,
            flash: p.FLASH,
            usb: p.USB_HS,
            power_bus,
            application: ApplicationBuilder {
                pins,
                uart1: p.UART1,
                uart2: p.UART2,
                spi2: p.SPI2,
                spi3: p.SPI3,
                sdhost: p.SDHOST,
                psram: p.PSRAM,
                cpu: p.CPU_CTRL,
                worker_interrupt: p.FROM_CPU_INTR1,
                io_mux: p.IO_MUX,
            },
        }
    }
}

impl ApplicationBuilder {
    /// Recovery never calls this: it retains only the boot/power resources.
    pub(crate) fn initialize(self) -> Application {
        // GPIO wakeups must preempt synchronous I2C recovery too, otherwise a
        // higher-priority acquisition executor alone cannot meet DRDY deadlines.
        Io::new(self.io_mux).set_interrupt_priority(SENSOR_INTERRUPT_PRIORITY);
        let mut pins = self.pins;
        let pps = Input::new(
            pins.take(wiring::gnss::PPS),
            InputConfig::default().with_pull(Pull::Down),
        );
        let ready = Input::new(pins.take(wiring::imu::DRDY), InputConfig::default());
        let command = native::init_gnss_com1(
            self.uart1,
            pins.take(wiring::gnss::COM1_MCU_TX)
                .try_into()
                .expect("HAL COM1 TX wiring"),
            pins.take(wiring::gnss::COM1_MCU_RX)
                .try_into()
                .expect("HAL COM1 RX wiring"),
            GnssUartSettings::default(),
        )
        .expect("GNSS COM1");
        let data = native::init_gnss_com2(
            self.uart2,
            pins.take(wiring::gnss::COM2_MCU_TX)
                .try_into()
                .expect("HAL COM2 TX wiring"),
            pins.take(wiring::gnss::COM2_MCU_RX)
                .try_into()
                .expect("HAL COM2 RX wiring"),
            GnssUartSettings::default(),
        )
        .expect("GNSS COM2");
        let gnss_reset = Output::new(
            pins.take(wiring::gnss::RESET_N),
            Level::High,
            OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
        );
        let spi = platform::ImuBus::new(
            self.spi2,
            pins.take(wiring::imu::SCLK)
                .try_into()
                .expect("HAL IMU SCLK wiring"),
            pins.take(wiring::imu::MOSI)
                .try_into()
                .expect("HAL IMU MOSI wiring"),
            pins.take(wiring::imu::MISO)
                .try_into()
                .expect("HAL IMU MISO wiring"),
        )
        .expect("IMU SPI");
        let imu = Imu::new(
            spi,
            pins.output(wiring::imu::CS_N, Level::High),
            pins.output(wiring::imu::RESET_N, Level::High),
            ready,
        );
        let display_spi = platform::DisplayBus::new(
            self.spi3,
            pins.take(wiring::display::SCLK)
                .try_into()
                .expect("HAL display SCLK wiring"),
            pins.take(wiring::display::MOSI)
                .try_into()
                .expect("HAL display MOSI wiring"),
        )
        .expect("display SPI");
        let display = Display::new(
            display_spi,
            pins.output(wiring::display::CS_N, Level::High),
            pins.output(wiring::display::DATA_COMMAND, Level::Low),
            pins.output(wiring::display::RESET_N, Level::High),
            pins.output(wiring::display::BACKLIGHT, Level::Low),
        );
        let led_enable = pins.output(wiring::power::LED_ENABLE, Level::Low);
        // Native four-bit SDHOST slot 0. Its owner lives for the application;
        // only the recorder worker issues card commands or accesses DMA memory.
        let sd = SD_CONTROLLER.init(
            esp_hal::sdmmc::SdHostController::new(self.sdhost, Default::default())
                .expect("SDHOST configuration"),
        );
        let recording_slot = sd
            .slot::<0>(Default::default())
            .expect("SD slot")
            .with_clk(GPIO24::try_from(pins.take(wiring::sd::CLK)).expect("SD CLK wiring"))
            .with_cmd(GPIO25::try_from(pins.take(wiring::sd::CMD)).expect("SD CMD wiring"))
            .with_data0(GPIO20::try_from(pins.take(wiring::sd::DATA[0])).expect("SD D0 wiring"))
            .with_data1(GPIO21::try_from(pins.take(wiring::sd::DATA[1])).expect("SD D1 wiring"))
            .with_data2(GPIO22::try_from(pins.take(wiring::sd::DATA[2])).expect("SD D2 wiring"))
            .with_data3(GPIO23::try_from(pins.take(wiring::sd::DATA[3])).expect("SD D3 wiring"));
        // MAX16169 active-low kill is released; retain its owner for application life.
        let power_hold = Output::new(
            pins.take(wiring::power::KILL_N),
            Level::High,
            OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
        );
        let memory = platform::initialize(self.psram);
        Application {
            sensors: SensorResources {
                imu,
                command,
                data,
                gnss_reset,
                pps,
            },
            display,
            maintenance: MaintenanceResources {
                led_enable,
                power_hold,
            },
            memory,
            cpu: self.cpu,
            worker_interrupt: self.worker_interrupt,
            recording_slot,
        }
    }
}

/// Install UART handlers only after the ports reach their acquisition executor.
/// The SDK's default interrupt priority is below power maintenance; promote the
/// existing handlers so a slow I2C transaction cannot prevent UART wakeups.
pub(crate) fn activate_gnss(
    command: Uart<'static, Blocking>,
    data: Uart<'static, Blocking>,
) -> (Uart<'static, Async>, Uart<'static, Async>) {
    let ports = (command.into_async(), data.into_async());
    interrupt::enable(Interrupt::UART1, SENSOR_INTERRUPT_PRIORITY);
    interrupt::enable(Interrupt::UART2, SENSOR_INTERRUPT_PRIORITY);
    ports
}
