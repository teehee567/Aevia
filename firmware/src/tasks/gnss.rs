use crate::{
    drivers::um980,
    state::{self, Measurement, Phase},
};
use aevia_firmware::gnss::{LineDecoder, ParseError, parse_bestnava, parse_bestnava_status};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{Async, uart::Uart};

pub async fn run(command: &mut Uart<'_, Async>, data: &mut Uart<'_, Async>) {
    loop {
        state::GNSS.update(|status| status.phase = Phase::Configuring);
        if let Err(error) = um980::configure(command, data).await {
            failed(error);
            Timer::after_secs(3).await;
            continue;
        }
        state::GNSS_ERROR.update(|last| *last = None);
        let mut decoder = LineDecoder::<1024>::new();
        let mut chunk = [0; 512];
        let mut last_record = Instant::now();
        loop {
            match with_timeout(
                Duration::from_secs(1),
                embedded_io_async::Read::read(data, &mut chunk),
            )
            .await
            {
                Ok(Ok(length)) if length != 0 => {
                    for byte in &chunk[..length] {
                        let Some(line) = decoder.push(*byte) else {
                            continue;
                        };
                        if !line.starts_with(b"#BESTNAVA,") {
                            continue;
                        }
                        let Ok(status) = parse_bestnava_status(line) else {
                            failed("invalid-frame");
                            continue;
                        };
                        let received_us = Instant::now().as_micros();
                        last_record = Instant::now();
                        let valid = match parse_bestnava(line) {
                            Ok(_) => true,
                            Err(ParseError::NoSolution) => false,
                            Err(_) => {
                                failed("invalid-solution");
                                continue;
                            }
                        };
                        state::GNSS.update(|state| {
                            state.phase = Phase::Ready;
                            state.records = state.records.wrapping_add(1);
                            state.valid = state.valid.wrapping_add(u32::from(valid));
                            state.last_us = Some(received_us);
                        });
                        state::FIX.update(|fix| {
                            fix.clear();
                            let _ = fix.push_str(
                                core::str::from_utf8(status.position_type).unwrap_or("unknown"),
                            );
                        });
                        if let Ok(text) = core::str::from_utf8(line)
                            && let Ok(line) = heapless::String::try_from(text.trim_end())
                        {
                            state::enqueue(Measurement::Gnss { received_us, line });
                        }
                    }
                }
                Ok(Err(_)) => {
                    failed("uart-read");
                    Timer::after_millis(10).await;
                }
                _ => {
                    Timer::after_millis(10).await;
                }
            }
            // Junk/CRC failures count as no receiver progress, even if the UART
            // never goes idle. Reconfigure with backoff after a bounded interval.
            if last_record.elapsed() >= Duration::from_secs(3) {
                failed("no-valid-records");
                state::FIX.update(|fix| fix.clear());
                break;
            }
        }
        Timer::after_secs(3).await;
    }
}

fn failed(error: &'static str) {
    state::GNSS.update(|status| {
        status.phase = Phase::Fault;
        status.errors = status.errors.wrapping_add(1);
    });
    state::GNSS_ERROR.update(|last| *last = Some(error));
}
