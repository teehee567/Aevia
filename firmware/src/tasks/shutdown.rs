use embassy_time::Timer;
use esp_hal::gpio::{Input, Output};

/// MAX16169AALT debounces SW7 and sends a 32 ms INT pulse. Ignore its
/// startup pulse and CLR blanking period before arming the next press.
#[embassy_executor::task]
pub async fn run(mut interrupt_n: Input<'static>, mut kill_n: Output<'static>) {
    Timer::after_millis(100).await;
    interrupt_n.wait_for_high().await;
    interrupt_n.wait_for_low().await;
    // There are no storage writes in base firmware. When recording is added,
    // a bounded storage flush belongs here, before removing power.
    kill_n.set_low();
    core::future::pending::<()>().await;
}
