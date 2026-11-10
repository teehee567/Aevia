//! ST7789VW four-wire SPI RGB565 display.
//!
//! Panel source: `data/datasheets/4520_C13462__________.pdf`, ZJY133T-IF05
//! Rev. B §§1.2, 3 and 4 (240×240, ST7789VW, active-low reset, four-wire SPI).
//! It specifies the panel and pinout, not the controller command set. Command
//! numbers and the 240×320 address space are additionally evidenced by the
//! [official Adafruit driver](https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp)
//! and its ST77xx header. The Mini preset preserves the existing board driver's
//! MADCTL=0, RGB, inversion-on and row-80 crop; rotation offsets are derived
//! from that crop rather than copied from another panel's default orientation.
//!
//! Configure SPI mode 0 and a board-qualified clock (existing Mini uses 24 MHz).
//! The SPI device owns active-low CS; DC is low for commands and high for data.
//! CS may be released between complete command/data bytes and pixel chunks.
//! Reset and backlight pins are separate, active-low and active-high respectively.
//! No constructor performs I/O. Initialization leaves the backlight off and does
//! not clear undefined frame RAM: fill it before explicitly enabling backlight.
//!
//! Transfers use at most 64 pixel-data bytes per transaction and no framebuffer.
//! A full rectangle still blocks until complete; call `write_row`/window methods
//! from a scheduler that gives sensor acquisition its required deadlines. TE is
//! available via `enable_tearing`, with edge capture/scheduling owned by caller.
//! The fitted connection has no readback wire, so successful writes cannot
//! confirm controller identity or electrical operation.

use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::SpiDevice};

pub const CONTROLLER_WIDTH: u16 = 240;
pub const CONTROLLER_HEIGHT: u16 = 320;
pub const PIXELS_PER_CHUNK: usize = 32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rotation {
    Deg0,
    Deg90,
    Deg180,
    Deg270,
}
impl Rotation {
    const fn bits(self) -> u8 {
        match self {
            Self::Deg0 => 0,
            Self::Deg90 => 0x60,
            Self::Deg180 => 0xc0,
            Self::Deg270 => 0xa0,
        }
    }
    const fn swaps_axes(self) -> bool {
        matches!(self, Self::Deg90 | Self::Deg270)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColorOrder {
    Rgb,
    Bgr,
}
impl ColorOrder {
    const fn bits(self) -> u8 {
        match self {
            Self::Rgb => 0,
            Self::Bgr => 8,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryError {
    Empty,
    OutOfBounds,
}

/// Native unrotated visible crop within the controller's 240×320 frame RAM.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Geometry {
    width: u16,
    height: u16,
    x_offset: u16,
    y_offset: u16,
}
impl Geometry {
    pub const fn new(
        width: u16,
        height: u16,
        x_offset: u16,
        y_offset: u16,
    ) -> Result<Self, GeometryError> {
        if width == 0 || height == 0 {
            return Err(GeometryError::Empty);
        }
        if width > CONTROLLER_WIDTH
            || height > CONTROLLER_HEIGHT
            || x_offset > CONTROLLER_WIDTH - width
            || y_offset > CONTROLLER_HEIGHT - height
        {
            return Err(GeometryError::OutOfBounds);
        }
        Ok(Self {
            width,
            height,
            x_offset,
            y_offset,
        })
    }
    pub const fn v2_mini() -> Self {
        Self {
            width: 240,
            height: 240,
            x_offset: 0,
            y_offset: 80,
        }
    }
    pub const fn size(self, rotation: Rotation) -> (u16, u16) {
        if rotation.swaps_axes() {
            (self.height, self.width)
        } else {
            (self.width, self.height)
        }
    }
    pub const fn offsets(self, rotation: Rotation) -> (u16, u16) {
        match rotation {
            Rotation::Deg0 => (self.x_offset, self.y_offset),
            Rotation::Deg90 => (self.y_offset, CONTROLLER_WIDTH - self.x_offset - self.width),
            Rotation::Deg180 => (
                CONTROLLER_WIDTH - self.x_offset - self.width,
                CONTROLLER_HEIGHT - self.y_offset - self.height,
            ),
            Rotation::Deg270 => (
                CONTROLLER_HEIGHT - self.y_offset - self.height,
                self.x_offset,
            ),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub geometry: Geometry,
    pub rotation: Rotation,
    pub color_order: ColorOrder,
    pub inversion: bool,
}
impl Config {
    pub const fn v2_mini() -> Self {
        Self {
            geometry: Geometry::v2_mini(),
            rotation: Rotation::Deg0,
            color_order: ColorOrder::Rgb,
            inversion: true,
        }
    }
    const fn madctl(self) -> u8 {
        self.rotation.bits() | self.color_order.bits()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Rect {
    pub x: u16,
    pub y: u16,
    pub width: u16,
    pub height: u16,
}
impl Rect {
    pub const fn new(x: u16, y: u16, width: u16, height: u16) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }
    fn count(self) -> usize {
        usize::from(self.width) * usize::from(self.height)
    }
    fn validate(self, size: (u16, u16)) -> Result<(), GeometryError> {
        if self.width == 0 || self.height == 0 {
            return Err(GeometryError::Empty);
        }
        if self.width > size.0
            || self.height > size.1
            || self.x > size.0 - self.width
            || self.y > size.1 - self.height
        {
            return Err(GeometryError::OutOfBounds);
        }
        Ok(())
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<S, DC, R, B> {
    Spi(S),
    DataCommand(DC),
    Reset(R),
    Backlight(B),
    Geometry(GeometryError),
    PixelCount,
    NotInitialized,
    Sleeping,
    NoWindow,
}

type DeviceResult<T, S, DC, R, B> = Result<T, Error<S, DC, R, B>>;

/// Owned device, control pins and delay. On a transfer/control error the driver
/// invalidates initialization and the active pixel window; reinitialize before
/// further typed operations because the controller's accepted bytes are unknown.
pub struct St7789<S, DC, R, B, D> {
    spi: S,
    dc: DC,
    reset: R,
    backlight: B,
    delay: D,
    config: Config,
    initialized: bool,
    sleeping: bool,
    remaining: Option<usize>,
}
impl<S, DC, R, B, D> St7789<S, DC, R, B, D> {
    pub const fn new(spi: S, dc: DC, reset: R, backlight: B, delay: D, config: Config) -> Self {
        Self {
            spi,
            dc,
            reset,
            backlight,
            delay,
            config,
            initialized: false,
            sleeping: false,
            remaining: None,
        }
    }
    pub fn release(self) -> (S, DC, R, B, D) {
        (self.spi, self.dc, self.reset, self.backlight, self.delay)
    }
    pub const fn configuration(&self) -> Config {
        self.config
    }
    pub const fn size(&self) -> (u16, u16) {
        self.config.geometry.size(self.config.rotation)
    }
    pub const fn initialized(&self) -> bool {
        self.initialized
    }
    pub const fn remaining_pixels(&self) -> Option<usize> {
        self.remaining
    }
    fn invalidate(&mut self) {
        self.initialized = false;
        self.remaining = None;
    }
}
impl<S: SpiDevice, DC: OutputPin, R: OutputPin, B: OutputPin, D: DelayNs> St7789<S, DC, R, B, D> {
    fn ready(&self) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        if !self.initialized {
            Err(Error::NotInitialized)
        } else if self.sleeping {
            Err(Error::Sleeping)
        } else {
            Ok(())
        }
    }
    fn dc_level(
        &mut self,
        data: bool,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        let result = if data {
            self.dc.set_high()
        } else {
            self.dc.set_low()
        };
        result.map_err(|e| {
            self.invalidate();
            Error::DataCommand(e)
        })
    }
    fn spi_write(
        &mut self,
        bytes: &[u8],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.spi.write(bytes).map_err(|e| {
            self.invalidate();
            Error::Spi(e)
        })
    }
    fn command(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.remaining = None;
        self.dc_level(false)?;
        self.spi_write(&[command])?;
        if !data.is_empty() {
            self.dc_level(true)?;
            self.spi_write(data)?;
        }
        Ok(())
    }
    fn data(&mut self, data: &[u8]) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.dc_level(true)?;
        self.spi_write(data)
    }
    /// Hardware reset + conservative existing-board delays, software reset,
    /// sleep exit, RGB565, configured orientation/inversion and display on.
    /// Leaves backlight off. No bus access is made by the constructor.
    pub fn initialize(&mut self) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.invalidate();
        self.backlight.set_low().map_err(Error::Backlight)?;
        self.reset.set_low().map_err(Error::Reset)?;
        self.delay.delay_ms(20);
        self.reset.set_high().map_err(Error::Reset)?;
        self.delay.delay_ms(120);
        self.command(0x01, &[])?;
        self.delay.delay_ms(150);
        self.command(0x11, &[])?;
        self.delay.delay_ms(120);
        self.command(0x3a, &[0x55])?;
        self.command(0x36, &[self.config.madctl()])?;
        self.command(if self.config.inversion { 0x21 } else { 0x20 }, &[])?;
        self.command(0x13, &[])?;
        self.command(0x29, &[])?;
        self.delay.delay_ms(10);
        self.initialized = true;
        self.sleeping = false;
        Ok(())
    }
    pub fn set_backlight(
        &mut self,
        enabled: bool,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        if enabled {
            self.ready()?;
        }
        let result = if enabled {
            self.backlight.set_high()
        } else {
            self.backlight.set_low()
        };
        result.map_err(|e| {
            self.invalidate();
            Error::Backlight(e)
        })
    }
    pub fn set_display_enabled(
        &mut self,
        enabled: bool,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.command(if enabled { 0x29 } else { 0x28 }, &[])
    }
    pub fn set_inversion(
        &mut self,
        enabled: bool,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.command(if enabled { 0x21 } else { 0x20 }, &[])?;
        self.config.inversion = enabled;
        Ok(())
    }
    pub fn set_rotation(
        &mut self,
        rotation: Rotation,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.command(0x36, &[rotation.bits() | self.config.color_order.bits()])?;
        self.config.rotation = rotation;
        Ok(())
    }
    pub fn set_color_order(
        &mut self,
        color_order: ColorOrder,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.command(0x36, &[self.config.rotation.bits() | color_order.bits()])?;
        self.config.color_order = color_order;
        Ok(())
    }
    pub fn sleep(&mut self) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.set_backlight(false)?;
        self.command(0x10, &[])?;
        self.delay.delay_ms(120);
        self.sleeping = true;
        Ok(())
    }
    pub fn wake(&mut self) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }
        self.command(0x11, &[])?;
        self.delay.delay_ms(120);
        self.sleeping = false;
        Ok(())
    }
    /// Enable TE output for vertical blank only (`true`) or turn it off. The
    /// GPIO input and deadline-aware synchronization are owned by the caller.
    pub fn enable_tearing(
        &mut self,
        enabled: bool,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        self.command(
            if enabled { 0x35 } else { 0x34 },
            if enabled { &[0] } else { &[] },
        )
    }
    /// Send a caller-validated controller command. This is the escape hatch
    /// for gamma, power/frame timing, scrolling and other documented settings.
    /// Caller must check the exact controller manual and preserve pixel-format
    /// and addressing assumptions. Known reset/sleep/display/MADCTL/COLMOD
    /// commands invalidate typed initialization; they need their typed helpers
    /// or a new `initialize`. Every raw command cancels the pixel window.
    pub fn raw_command(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        if matches!(command, 0x01 | 0x10 | 0x11 | 0x28 | 0x29 | 0x36 | 0x3a) {
            self.invalidate();
        }
        self.command(command, data)
    }
    /// Select a visible rectangle and start RAMWR. Complete it with one or
    /// more pixel writes before sending another command. At most the remaining
    /// pixel count can be submitted. Any bus/control error invalidates it.
    pub fn begin_window(
        &mut self,
        rect: Rect,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        rect.validate(self.size()).map_err(Error::Geometry)?;
        self.ready()?;
        let (ox, oy) = self.config.geometry.offsets(self.config.rotation);
        let x0 = (rect.x + ox).to_be_bytes();
        let x1 = (rect.x + ox + rect.width - 1).to_be_bytes();
        let y0 = (rect.y + oy).to_be_bytes();
        let y1 = (rect.y + oy + rect.height - 1).to_be_bytes();
        self.command(0x2a, &[x0[0], x0[1], x1[0], x1[1]])?;
        self.command(0x2b, &[y0[0], y0[1], y1[0], y1[1]])?;
        self.command(0x2c, &[])?;
        self.remaining = Some(rect.count());
        Ok(())
    }
    /// Stream native RGB565 words in controller byte order (most significant
    /// byte first), through a 64-byte scratch buffer.
    pub fn write_pixels(
        &mut self,
        pixels: &[u16],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.ready()?;
        let remaining = self.remaining.ok_or(Error::NoWindow)?;
        if pixels.len() > remaining {
            return Err(Error::PixelCount);
        }
        let mut bytes = [0; PIXELS_PER_CHUNK * 2];
        for chunk in pixels.chunks(PIXELS_PER_CHUNK) {
            for (dst, pixel) in bytes.as_chunks_mut::<2>().0.iter_mut().zip(chunk) {
                dst.copy_from_slice(&pixel.to_be_bytes());
            }
            self.data(&bytes[..chunk.len() * 2])?;
            self.remaining = Some(self.remaining.unwrap() - chunk.len());
        }
        Ok(())
    }
    pub fn write_rect(
        &mut self,
        rect: Rect,
        pixels: &[u16],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        rect.validate(self.size()).map_err(Error::Geometry)?;
        if pixels.len() != rect.count() {
            return Err(Error::PixelCount);
        }
        self.begin_window(rect)?;
        self.write_pixels(pixels)
    }
    /// Submit already big-endian RGB565 bytes, also in bounded 64-byte chunks.
    pub fn write_rect_bytes(
        &mut self,
        rect: Rect,
        bytes: &[u8],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        rect.validate(self.size()).map_err(Error::Geometry)?;
        if !bytes.len().is_multiple_of(2) || bytes.len() / 2 != rect.count() {
            return Err(Error::PixelCount);
        }
        self.begin_window(rect)?;
        for chunk in bytes.chunks(PIXELS_PER_CHUNK * 2) {
            self.data(chunk)?;
            self.remaining = Some(self.remaining.unwrap() - chunk.len() / 2);
        }
        Ok(())
    }
    /// Draw one horizontal run. Short rows are useful scheduling units.
    pub fn write_row(
        &mut self,
        x: u16,
        y: u16,
        pixels: &[u16],
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        let width =
            u16::try_from(pixels.len()).map_err(|_| Error::Geometry(GeometryError::OutOfBounds))?;
        self.write_rect(Rect::new(x, y, width, 1), pixels)
    }
    pub fn set_pixel(
        &mut self,
        x: u16,
        y: u16,
        pixel: u16,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.write_rect(Rect::new(x, y, 1, 1), &[pixel])
    }
    pub fn fill_rect(
        &mut self,
        rect: Rect,
        color: u16,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        self.begin_window(rect)?;
        let mut bytes = [0; PIXELS_PER_CHUNK * 2];
        for dst in bytes.as_chunks_mut::<2>().0 {
            dst.copy_from_slice(&color.to_be_bytes());
        }
        while let Some(remaining) = self.remaining {
            if remaining == 0 {
                break;
            }
            let count = remaining.min(PIXELS_PER_CHUNK);
            self.data(&bytes[..count * 2])?;
            self.remaining = Some(remaining - count);
        }
        Ok(())
    }
    pub fn clear(
        &mut self,
        color: u16,
    ) -> DeviceResult<(), S::Error, DC::Error, R::Error, B::Error> {
        let (w, h) = self.size();
        self.fill_rect(Rect::new(0, 0, w, h), color)
    }
}

#[cfg(test)]
mod tests;
