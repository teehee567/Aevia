//! ST7789VW panel transport and initialization; scheduling/rendering live in `tasks::display`.
use embassy_time::Timer;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Flex, InputConfig, Output, OutputConfig, Pin, Pull},
    spi::{
        Error,
        master::{Address, Command, DataMode, Spi},
    },
};

#[derive(Clone, Copy, Default)]
pub struct Readback {
    pub id: u32,
    pub registers: u32,
}
/// Run before SPI owns SCLK. Only drive the pin if both weak pulls work.
pub fn check_sclk<'d>(pin: impl Pin + 'd) -> u32 {
    let mut pin = Flex::new(pin);
    pin.set_output_enable(false);
    pin.set_input_enable(true);
    pin.apply_input_config(&InputConfig::default().with_pull(Pull::Down));
    Delay::new().delay_micros(1000);
    let down = u32::from(pin.is_high());
    pin.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
    Delay::new().delay_micros(1000);
    let up = u32::from(pin.is_high());
    pin.apply_input_config(&InputConfig::default());
    let mut result = down | (up << 1);
    if down == 0 && up == 1 {
        pin.apply_output_config(&OutputConfig::default());
        pin.set_low();
        pin.set_output_enable(true);
        Delay::new().delay_micros(2);
        let low = u32::from(pin.is_high());
        pin.set_high();
        Delay::new().delay_micros(2);
        let high = u32::from(pin.is_high());
        pin.set_low();
        pin.set_output_enable(false);
        result |= (low << 2) | (high << 3) | 16;
    }
    // Bits: pull-down input, pull-up input, driven-low input, driven-high
    // input, output test performed. Healthy = 26 (0x1a).
    result
}
pub struct Lcd {
    spi: Spi<'static, Blocking>,
    cs: Output<'static>,
    dc: Flex<'static>,
    reset: Output<'static>,
}

impl Lcd {
    pub fn check_dc(&mut self) -> u32 {
        self.cs.set_high();
        self.dc.set_low();
        Delay::new().delay_micros(2);
        let low = u32::from(self.dc.is_high());
        self.dc.set_high();
        Delay::new().delay_micros(2);
        let high = u32::from(self.dc.is_high());
        let latch = u32::from(self.dc.is_set_high());
        self.dc.set_low();
        // Expected 5: output latch high and sensed high, sensed low while low.
        (latch << 2) | (low << 1) | high
    }

    fn read_register<const N: usize>(&mut self, command: u8, dummy: u8) -> Result<[u8; N], Error> {
        let mut bytes = [0; N];
        self.cs.set_low();
        self.dc.set_low();
        // Shared SDA must be released at the end of the command byte. The
        // hardware half-duplex command phase performs that turnaround.
        let result = self.spi.half_duplex_read(
            DataMode::Single,
            Command::_8Bit(command as u16, DataMode::Single),
            Address::None,
            dummy,
            &mut bytes,
        );
        self.cs.set_high();
        result.map(|()| bytes)
    }

    pub fn readback(&mut self) -> Result<Readback, Error> {
        // ST7789VW section 8.4.5: RDDID needs one dummy clock; 8-bit reads do not.
        let id = self.read_register::<3>(0x04, 1)?;
        let power = self.read_register::<1>(0x0a, 0)?[0];
        let orientation = self.read_register::<1>(0x0b, 0)?[0];
        let color = self.read_register::<1>(0x0c, 0)?[0];
        let signal = self.read_register::<1>(0x0e, 0)?[0];
        Ok(Readback {
            id: u32::from_be_bytes([0, id[0], id[1], id[2]]),
            registers: u32::from_be_bytes([power, orientation, color, signal]),
        })
    }

    pub fn new(
        spi: Spi<'static, Blocking>,
        cs: Output<'static>,
        dc: Output<'static>,
        reset: Output<'static>,
    ) -> Self {
        let mut dc = dc.into_flex();
        dc.set_input_enable(true);
        Self { spi, cs, dc, reset }
    }

    fn command(&mut self, command: u8, data: &[u8]) -> Result<(), Error> {
        self.cs.set_low();
        self.dc.set_low();
        Delay::new().delay_micros(2);
        let result = self
            .spi
            .half_duplex_write(
                DataMode::Single,
                Command::_8Bit(command as u16, DataMode::Single),
                Address::None,
                0,
                &[],
            )
            .and_then(|()| {
                self.dc.set_high();
                Delay::new().delay_micros(2);
                if data.is_empty() {
                    Ok(())
                } else {
                    self.spi.half_duplex_write(
                        DataMode::Single,
                        Command::None,
                        Address::None,
                        0,
                        data,
                    )
                }
            });
        self.cs.set_high();
        result
    }

    pub async fn initialize(&mut self) -> Result<(), Error> {
        self.cs.set_high();
        self.reset.set_high();
        Timer::after_millis(10).await;
        self.reset.set_low();
        Timer::after_millis(20).await;
        self.reset.set_high();
        Timer::after_millis(150).await;
        // Standard ST7789 sequence, with conservative reset/sleep-out delays.
        // https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp
        self.command(0x01, &[])?; // SWRESET
        Timer::after_millis(150).await;
        self.command(0x11, &[])?; // SLPOUT
        Timer::after_millis(120).await;
        self.command(0x3a, &[0x55])?; // COLMOD: RGB565
        // Adafruit rotation 0: mirror X/Y, visible RAM rows 80..319.
        self.command(0x36, &[0xc0])?;
        self.command(0x21, &[])?; // INVON: normally-black IPS panel
        self.command(0x13, &[])?; // NORON
        self.command(0x35, &[0x00])?; // TEON: vertical blank only
        Timer::after_millis(10).await;
        Ok(())
    }

    pub fn display_on(&mut self) -> Result<(), Error> {
        self.command(0x29, &[])
    }

    /// Render a row in SRAM, then yield between 64-byte transfers (~512 us).
    /// A whole blocking row exceeds the GNSS FIFO's service-time budget.
    pub async fn write_frame(&mut self, render: impl Fn(u16, &mut [u8; 480])) -> Result<(), Error> {
        self.command(0x2a, &[0, 0, 0, 239])?;
        self.command(0x2b, &[0, 80, 1, 63])?;
        self.command(0x2c, &[])?;
        let mut row = [0; 480];
        for y in 0..240 {
            render(y, &mut row);
            for chunk in row.chunks(64) {
                self.cs.set_low();
                self.dc.set_high();
                let result = self.spi.half_duplex_write(
                    DataMode::Single,
                    Command::None,
                    Address::None,
                    0,
                    chunk,
                );
                self.cs.set_high();
                result?;
                Timer::after_micros(1).await;
            }
        }
        Ok(())
    }
}
