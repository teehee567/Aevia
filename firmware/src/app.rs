//! Startup composition. Hardware faults remain local to their owning task.
use crate::{board, state, tasks};
use embassy_executor::Spawner;
use esp_hal::{
    psram::{Psram, PsramConfig, PsramTimingParams},
    system::Stack,
    timer::timg::TimerGroup,
};

static CORE1_STACK: static_cell::ConstStaticCell<Stack<16384>> =
    static_cell::ConstStaticCell::new(Stack::new());

pub async fn run(spawner: Spawner) {
    let board = board::initialize();
    let timer = TimerGroup::new(board.timer);
    esp_rtos::start(timer.timer0, board.core0_interrupt);

    spawner.spawn(
        tasks::shutdown::run(board.power_interrupt_n, board.power_kill_n).expect("shutdown task"),
    );
    spawner.spawn(
        tasks::power::task(
            board.power_bus,
            board.led_enable,
            board.charger_power_good_n,
        )
        .expect("power task"),
    );
    spawner.spawn(
        tasks::display::run(board.lcd, board.lcd_te, board.lcd_sclk_probe).expect("display task"),
    );
    spawner.spawn(tasks::display::backlight::run(board.backlight).expect("backlight task"));

    let psram = Psram::new(
        board.psram,
        PsramConfig {
            timing: PsramTimingParams::MHZ_125,
            ..Default::default()
        },
    );
    esp_rtos::start_second_core(
        board.cpu,
        board.core1_interrupt,
        CORE1_STACK.take(),
        move || {
            // Memory is exclusively owned here and never backs stacks or USB buffers.
            // A failed memory check does not prevent sensor/USB/power bring-up.
            let result = aevia_firmware::memory::bring_up(psram);
            state::MEMORY.update(|status| *status = result.map_err(Some));
            static EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::Executor> =
                static_cell::StaticCell::new();
            EXECUTOR
                .init(esp_rtos::embassy::Executor::new())
                .run(|spawner| {
                    spawner.spawn(tasks::imu::run(board.imu).expect("IMU task"));
                });
        },
    );
    crate::usb::run(
        spawner,
        board.usb,
        board.gnss_command,
        board.gnss_data,
        board.gnss_reset_n,
    )
    .await;
}
