use crate::{
    drivers::sch16t::Imu,
    state::{self, Measurement, Phase},
};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn run(mut sensor: Imu<'static>) {
    loop {
        state::IMU.update(|status| status.phase = Phase::Configuring);
        if let Err(error) = sensor.initialize().await {
            failed(error);
            Timer::after_secs(2).await;
            continue;
        }
        let mut consecutive_errors = 0;
        loop {
            match sensor.next_sample().await {
                Ok(sample) => {
                    consecutive_errors = 0;
                    state::IMU.update(|status| {
                        status.phase = Phase::Ready;
                        status.records = status.records.wrapping_add(1);
                        status.last_us = Some(sample.monotonic_us);
                    });
                    state::enqueue(Measurement::Imu(sample));
                }
                Err(error) => {
                    failed(error);
                    consecutive_errors += 1;
                    if consecutive_errors >= 10 {
                        break;
                    }
                    // A pin stuck high must not turn repeated bad frames into
                    // an unbounded executor poll that starves other work.
                    Timer::after_millis(1).await;
                }
            }
        }
        Timer::after_secs(2).await;
    }
}

fn failed(error: aevia_firmware::imu::ImuError) {
    state::IMU.update(|status| {
        status.phase = Phase::Fault;
        status.errors = status.errors.wrapping_add(1);
    });
    state::IMU_ERROR.update(|last| *last = Some(error));
}
