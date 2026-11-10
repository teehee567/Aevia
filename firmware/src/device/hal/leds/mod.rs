//! Optional serial RGB LEDs: XL-2020RGBC-WS2812B and APA102-2020.
//!
//! Mini's fitted MSL0402 LEDs are passive packages driven by `lp5813`; this
//! module must not be used as their backend. Earlier V2 lists APA102 devices.
//!
//! WS2812 source: local `data/datasheets/5962_XL-2020RGBC-WS2812B.pdf`,
//! pp.6–8: GRB, MSB first, reset low >80 µs, T0H<470 ns, T1H>580 ns and
//! <=1000 ns, bit period >=1200 ns. Each source bit becomes SPI `100` or `110`.
//! Actual SPI clock is checked against those limits; each complete frame has
//! at least100 µs low padding before and after the pixels. The caller supplies the
//! complete encoded frame, so this driver never inserts per-pixel write gaps.
//!
//! **WS2812 transport requirement:** configure SPI mode0/MSB-first and use a
//! qualified hardware/DMA backend that clocks the entire slice without pauses.
//! `SpiBus::write` alone does not promise this; ordinary FIFO refill, interrupts
//! or scheduler preemption can stretch high pulses and invalidate the waveform.
//! The caller must supply the actual clock, including divider/clock tolerances,
//! keep MOSI low after the final zero bit, and own the output exclusively. The
//! SPI clock wire is not connected to WS2812. No software delay loop is used.
//!
//! APA102 source: manufacturer [APA102-2020 sheet pp.3–4](https://cdn-shop.adafruit.com/product-files/3341/2412161026_Led-Color-APA102-2020-SuperLED_C22439302.pdf)
//! (Shenzhen LED Color), with scalable end clocks corroborated by the
//! [official Adafruit implementation](https://github.com/adafruit/Adafruit_DotStar/blob/master/Adafruit_DotStar.cpp).
//! Wire order is 32 zero start bits, per-pixel `111` + five-bit brightness +
//! blue/green/red, and high end bits. End padding is at least32 bits and at
//! least ceil(pixel_count/2) clocks. Brightness codes0..31 are supported;0 is off.
//! Set a board-qualified clock and mode0/MSB-first. Pauses are permitted for this
//! clocked protocol, but other SPI traffic is not: these LEDs have no CS input.
//!
//! Constructors and encoders perform no I/O, allocate no memory, and never
//! alter input colors. `show` validates/encodes the complete frame before bus
//! access. Empty frames are no-ops. A transport error can leave partially
//! displayed data; retry a complete frame. There is no readback/acknowledgment.

use embedded_hal::spi::SpiBus;

/// RGB channel values, independent of the device's wire order.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Rgb8 {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}
impl Rgb8 {
    pub const BLACK: Self = Self::new(0, 0, 0);
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EncodingError {
    InvalidSpiClock,
    InvalidBrightness,
    SizeOverflow,
    BufferTooSmall { required: usize, available: usize },
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Encoding(EncodingError),
    Bus(E),
    /// Retains both failures when draining the bus after a failed write fails.
    Transfer {
        write: E,
        flush: E,
    },
}

/// The clock actually delivered by the backend, not just its requested rate.
/// Use a rate whose entire clock tolerance remains inside the accepted range.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ws2812Timing {
    spi_hz: u32,
}
impl Ws2812Timing {
    pub const RESET_LOW_MICROSECONDS: u32 = 100;
    pub const MIN_SPI_HZ: u32 = 2_127_660;
    pub const MAX_SPI_HZ: u32 = 2_500_000;
    pub const fn new(actual_spi_hz: u32) -> Result<Self, EncodingError> {
        let hz = actual_spi_hz as u64;
        // Exact rational comparisons avoid rounding a boundary into range.
        if hz == 0
            || 1_000_000_000 >= 470 * hz
            || 2_000_000_000 <= 580 * hz
            || 2_000_000_000 > 1000 * hz
            || 3_000_000_000 < 1200 * hz
        {
            Err(EncodingError::InvalidSpiClock)
        } else {
            Ok(Self {
                spi_hz: actual_spi_hz,
            })
        }
    }
    pub const fn spi_hz(self) -> u32 {
        self.spi_hz
    }
    /// Zero bytes needed for at least100 µs low at this clock.
    pub const fn reset_bytes(self) -> usize {
        // Eight bits/byte,100 µs => ceil(hz / 80_000).
        self.spi_hz.div_ceil(80_000) as usize
    }
    pub fn encoded_len(self, pixel_count: usize) -> Result<usize, EncodingError> {
        if pixel_count == 0 {
            return Ok(0);
        }
        pixel_count
            .checked_mul(9)
            .and_then(|n| n.checked_add(self.reset_bytes() * 2))
            .ok_or(EncodingError::SizeOverflow)
    }
}

/// Encode one complete waveform; bytes beyond the returned length are untouched.
/// Rejection leaves the output buffer untouched. See module transport contract.
pub fn encode_ws2812(
    colors: &[Rgb8],
    timing: Ws2812Timing,
    output: &mut [u8],
) -> Result<usize, EncodingError> {
    let required = timing.encoded_len(colors.len())?;
    require_capacity(output.len(), required)?;
    if required == 0 {
        return Ok(0);
    }
    let padding = timing.reset_bytes();
    output[..padding].fill(0);
    let mut offset = padding;
    for color in colors {
        for channel in [color.green, color.red, color.blue] {
            let mut encoded = 0_u32;
            for bit in (0..8).rev() {
                encoded = (encoded << 3)
                    | if channel & (1 << bit) == 0 {
                        0b100
                    } else {
                        0b110
                    };
            }
            let bytes = encoded.to_be_bytes();
            output[offset..offset + 3].copy_from_slice(&bytes[1..]);
            offset += 3;
        }
    }
    output[offset..required].fill(0);
    Ok(required)
}

/// Owns a dedicated SPI data output with the waveform contract above.
pub struct Ws2812Spi<B> {
    bus: B,
    timing: Ws2812Timing,
}
impl<B> Ws2812Spi<B> {
    pub const fn new(bus: B, timing: Ws2812Timing) -> Self {
        Self { bus, timing }
    }
    pub const fn timing(&self) -> Ws2812Timing {
        self.timing
    }
    pub fn release(self) -> B {
        self.bus
    }
}
impl<B: SpiBus> Ws2812Spi<B> {
    pub fn show(&mut self, colors: &[Rgb8], frame: &mut [u8]) -> Result<(), Error<B::Error>> {
        let length = encode_ws2812(colors, self.timing, frame).map_err(Error::Encoding)?;
        transmit(&mut self.bus, &frame[..length])
    }
}

/// APA102's five-bit per-pixel global output level.0=off,31=full scale.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GlobalBrightness(u8);
impl GlobalBrightness {
    pub const OFF: Self = Self(0);
    pub const FULL: Self = Self(31);
    pub const fn new(code: u8) -> Result<Self, EncodingError> {
        if code <= 31 {
            Ok(Self(code))
        } else {
            Err(EncodingError::InvalidBrightness)
        }
    }
    pub const fn code(self) -> u8 {
        self.0
    }
}
impl Default for GlobalBrightness {
    fn default() -> Self {
        Self::FULL
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Apa102Pixel {
    pub color: Rgb8,
    pub brightness: GlobalBrightness,
}
impl Apa102Pixel {
    pub const fn new(color: Rgb8, brightness: GlobalBrightness) -> Self {
        Self { color, brightness }
    }
}

/// At least32 clocks per the sheet, extended for chains needing more clocks.
pub const fn apa102_end_bytes(pixel_count: usize) -> usize {
    let scalable = pixel_count.div_ceil(16);
    if scalable > 4 { scalable } else { 4 }
}
pub fn apa102_encoded_len(pixel_count: usize) -> Result<usize, EncodingError> {
    if pixel_count == 0 {
        return Ok(0);
    }
    pixel_count
        .checked_mul(4)
        .and_then(|n| n.checked_add(4))
        .and_then(|n| n.checked_add(apa102_end_bytes(pixel_count)))
        .ok_or(EncodingError::SizeOverflow)
}

/// Encode complete start/pixel/end frames. Rejects before modifying output.
pub fn encode_apa102(pixels: &[Apa102Pixel], output: &mut [u8]) -> Result<usize, EncodingError> {
    let required = apa102_encoded_len(pixels.len())?;
    require_capacity(output.len(), required)?;
    if required == 0 {
        return Ok(0);
    }
    output[..4].fill(0);
    let mut offset = 4;
    for pixel in pixels {
        output[offset..offset + 4].copy_from_slice(&[
            0xe0 | pixel.brightness.code(),
            pixel.color.blue,
            pixel.color.green,
            pixel.color.red,
        ]);
        offset += 4;
    }
    output[offset..required].fill(0xff);
    Ok(required)
}

/// Owns an unselected, dedicated SPI clock/data output pair (APA102 has no CS).
pub struct Apa102Spi<B> {
    bus: B,
}
impl<B> Apa102Spi<B> {
    pub const fn new(bus: B) -> Self {
        Self { bus }
    }
    pub fn release(self) -> B {
        self.bus
    }
}
impl<B: SpiBus> Apa102Spi<B> {
    pub fn show(
        &mut self,
        pixels: &[Apa102Pixel],
        frame: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        let length = encode_apa102(pixels, frame).map_err(Error::Encoding)?;
        transmit(&mut self.bus, &frame[..length])
    }
}

fn require_capacity(available: usize, required: usize) -> Result<(), EncodingError> {
    if available < required {
        Err(EncodingError::BufferTooSmall {
            required,
            available,
        })
    } else {
        Ok(())
    }
}
fn transmit<B: SpiBus>(bus: &mut B, bytes: &[u8]) -> Result<(), Error<B::Error>> {
    if bytes.is_empty() {
        return Ok(());
    }
    let sent = bus.write(bytes);
    let flushed = bus.flush();
    match (sent, flushed) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(write), Err(flush)) => Err(Error::Transfer { write, flush }),
        (Err(error), _) | (_, Err(error)) => Err(Error::Bus(error)),
    }
}

#[cfg(test)]
mod tests;
