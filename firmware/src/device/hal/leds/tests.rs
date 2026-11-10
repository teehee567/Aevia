use super::*;
use std::vec::Vec;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Fault {
    Write,
    Flush,
}
impl embedded_hal::spi::Error for Fault {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}
#[derive(Default)]
struct Bus {
    writes: Vec<Vec<u8>>,
    flushes: usize,
    fail_write: bool,
    fail_flush: bool,
}
impl embedded_hal::spi::ErrorType for Bus {
    type Error = Fault;
}
impl SpiBus for Bus {
    fn read(&mut self, _: &mut [u8]) -> Result<(), Fault> {
        panic!("unexpected read")
    }
    fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Fault> {
        panic!("unexpected transfer")
    }
    fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Fault> {
        panic!("unexpected transfer")
    }
    fn write(&mut self, data: &[u8]) -> Result<(), Fault> {
        self.writes.push(data.to_vec());
        if self.fail_write {
            Err(Fault::Write)
        } else {
            Ok(())
        }
    }
    fn flush(&mut self) -> Result<(), Fault> {
        self.flushes += 1;
        if self.fail_flush {
            Err(Fault::Flush)
        } else {
            Ok(())
        }
    }
}
fn timing() -> Ws2812Timing {
    Ws2812Timing::new(2_400_000).unwrap()
}

#[test]
fn ws_clock_boundaries_use_exact_datasheet_inequalities() {
    for bad in [
        0,
        2_000_000,
        Ws2812Timing::MIN_SPI_HZ - 1,
        Ws2812Timing::MAX_SPI_HZ + 1,
        u32::MAX,
    ] {
        assert_eq!(Ws2812Timing::new(bad), Err(EncodingError::InvalidSpiClock));
    }
    for hz in [
        Ws2812Timing::MIN_SPI_HZ,
        2_400_000,
        Ws2812Timing::MAX_SPI_HZ,
    ] {
        let t = Ws2812Timing::new(hz).unwrap();
        let period = 1_000_000_000.0 / f64::from(hz);
        assert!(period < 470.0);
        assert!(period * 2.0 > 580.0 && period * 2.0 <= 1000.0);
        assert!(period * 3.0 >= 1200.0);
        assert!(t.reset_bytes() as u64 * 8 * 1_000_000 >= u64::from(hz) * 100);
    }
}

#[test]
fn ws_known_zero_one_vectors_and_grb_order_are_exact() {
    let mut frame = [0xa5; 80];
    let n = encode_ws2812(&[Rgb8::new(255, 0, 0)], timing(), &mut frame).unwrap();
    assert_eq!(n, 69);
    assert_eq!(&frame[..30], &[0; 30]);
    assert_eq!(
        &frame[30..39],
        &[0x92, 0x49, 0x24, 0xdb, 0x6d, 0xb6, 0x92, 0x49, 0x24]
    );
    assert_eq!(&frame[39..n], &[0; 30]);
    assert_eq!(&frame[n..], &[0xa5; 11]);
}

#[test]
fn ws_every_source_byte_keeps_msb_order_and_low_symbol_terminators() {
    let mut frame = [0; 80];
    for value in 0..=255_u8 {
        encode_ws2812(&[Rgb8::new(value, 0x55, 0xaa)], timing(), &mut frame).unwrap();
        let wave = &frame[30..39];
        for (channel, expected) in [0x55_u8, value, 0xaa].into_iter().enumerate() {
            for source_bit in 0..8 {
                let mut symbol = 0;
                for i in 0..3 {
                    let bit = channel * 24 + source_bit * 3 + i;
                    symbol = (symbol << 1) | (wave[bit / 8] >> (7 - bit % 8)) & 1;
                }
                assert_eq!(
                    symbol,
                    if expected & (0x80 >> source_bit) == 0 {
                        0b100
                    } else {
                        0b110
                    }
                );
            }
        }
    }
}

#[test]
fn output_capacity_errors_do_not_mutate_or_transmit() {
    let mut small = [0xa5; 4];
    let before = small;
    assert_eq!(
        encode_ws2812(&[Rgb8::BLACK], timing(), &mut small),
        Err(EncodingError::BufferTooSmall {
            required: 69,
            available: 4
        })
    );
    assert_eq!(small, before);
    let mut ws = Ws2812Spi::new(Bus::default(), timing());
    assert!(matches!(
        ws.show(&[Rgb8::BLACK], &mut small),
        Err(Error::Encoding(_))
    ));
    assert!(ws.release().writes.is_empty());
    let mut apa = Apa102Spi::new(Bus::default());
    assert!(matches!(
        apa.show(&[Apa102Pixel::default()], &mut small),
        Err(Error::Encoding(_))
    ));
    assert!(apa.release().writes.is_empty());
    assert_eq!(small, before);
}

#[test]
fn frame_size_overflow_is_reported_without_allocation() {
    assert_eq!(
        timing().encoded_len(usize::MAX),
        Err(EncodingError::SizeOverflow)
    );
    assert_eq!(
        apa102_encoded_len(usize::MAX),
        Err(EncodingError::SizeOverflow)
    );
}

#[test]
fn apa_known_start_brightness_bgr_and_end_frame() {
    let pixel = Apa102Pixel::new(
        Rgb8::new(0x11, 0x22, 0x33),
        GlobalBrightness::new(3).unwrap(),
    );
    let mut output = [0xa5; 15];
    assert_eq!(encode_apa102(&[pixel], &mut output), Ok(12));
    assert_eq!(
        &output[..12],
        &[0, 0, 0, 0, 0xe3, 0x33, 0x22, 0x11, 0xff, 0xff, 0xff, 0xff]
    );
    assert_eq!(&output[12..], &[0xa5; 3]);
    let mut frame = [0; 16];
    encode_apa102(
        &[
            Apa102Pixel::new(Rgb8::new(1, 2, 3), GlobalBrightness::OFF),
            Apa102Pixel::new(Rgb8::new(4, 5, 6), GlobalBrightness::FULL),
        ],
        &mut frame,
    )
    .unwrap();
    assert_eq!(frame[4], 0xe0);
    assert_eq!(frame[8], 0xff);
}

#[test]
fn apa_brightness_accepts_exactly_five_bits() {
    for level in 0..=31 {
        assert_eq!(GlobalBrightness::new(level).unwrap().code(), level);
    }
    for level in 32..=255 {
        assert_eq!(
            GlobalBrightness::new(level),
            Err(EncodingError::InvalidBrightness)
        );
    }
}

#[test]
fn apa_end_clocks_scale_past_sixty_four_leds() {
    for (count, bytes) in [
        (1, 4),
        (16, 4),
        (64, 4),
        (65, 5),
        (128, 8),
        (129, 9),
        (1024, 64),
    ] {
        assert_eq!(apa102_end_bytes(count), bytes);
        assert!(bytes * 8 >= count.div_ceil(2));
        assert_eq!(apa102_encoded_len(count), Ok(4 + 4 * count + bytes));
    }
}

#[test]
fn complete_ws_frame_is_one_write_followed_by_flush() {
    let mut dev = Ws2812Spi::new(Bus::default(), timing());
    let colors = [Rgb8::new(1, 2, 3), Rgb8::new(4, 5, 6)];
    let mut frame = [0; 80];
    dev.show(&colors, &mut frame).unwrap();
    let bus = dev.release();
    assert_eq!(bus.writes.len(), 1);
    assert_eq!(bus.writes[0], frame[..78]);
    assert_eq!(bus.flushes, 1);
}

#[test]
fn apa_stream_is_contiguous_and_construction_is_silent() {
    assert!(Apa102Spi::new(Bus::default()).release().writes.is_empty());
    assert!(
        Ws2812Spi::new(Bus::default(), timing())
            .release()
            .writes
            .is_empty()
    );
    let mut dev = Apa102Spi::new(Bus::default());
    let mut frame = [0; 12];
    dev.show(&[Apa102Pixel::default()], &mut frame).unwrap();
    let bus = dev.release();
    assert_eq!(bus.writes, [frame.to_vec()]);
    assert_eq!(bus.flushes, 1);
}

#[test]
fn write_and_flush_failures_are_preserved_and_flush_always_attempted() {
    let mut dev = Ws2812Spi::new(
        Bus {
            fail_write: true,
            fail_flush: true,
            ..Bus::default()
        },
        timing(),
    );
    assert_eq!(
        dev.show(&[Rgb8::BLACK], &mut [0; 80]),
        Err(Error::Transfer {
            write: Fault::Write,
            flush: Fault::Flush
        })
    );
    assert_eq!(dev.release().flushes, 1);
    let mut dev = Apa102Spi::new(Bus {
        fail_flush: true,
        ..Bus::default()
    });
    assert_eq!(
        dev.show(&[Apa102Pixel::default()], &mut [0; 12]),
        Err(Error::Bus(Fault::Flush))
    );
    assert_eq!(dev.release().flushes, 1);
}

#[test]
fn empty_frames_are_noops_and_leave_scratch_untouched() {
    let mut scratch = [0xa5; 5];
    assert_eq!(encode_apa102(&[], &mut scratch), Ok(0));
    assert_eq!(encode_ws2812(&[], timing(), &mut scratch), Ok(0));
    let mut ws = Ws2812Spi::new(Bus::default(), timing());
    ws.show(&[], &mut scratch).unwrap();
    assert!(ws.release().writes.is_empty());
    let mut apa = Apa102Spi::new(Bus::default());
    apa.show(&[], &mut scratch).unwrap();
    assert!(apa.release().writes.is_empty());
    assert_eq!(scratch, [0xa5; 5]);
}
