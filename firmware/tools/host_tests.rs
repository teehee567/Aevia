//! Run exact device-independent sensor, power, storage and codec implementations.
#[path = "../src/sample.rs"]
pub mod sample;
#[path = "../src/recording.rs"]
pub mod recording;
#[path = "../src/response.rs"]
pub mod response;
#[path = "../src/gnss.rs"]
pub mod gnss;
#[path = "../src/gnss_binary.rs"]
pub mod gnss_binary;
#[path = "../src/ppk.rs"]
pub mod ppk;
#[path = "../src/uart_pipe.rs"]
pub mod uart_pipe;
#[path = "../src/preview_queue.rs"]
pub mod preview_queue;
#[path = "../src/imu.rs"]
pub mod imu;
#[path = "../src/buttons.rs"]
pub mod buttons;
#[path = "../src/battery.rs"]
pub mod battery;
#[path = "../src/storage/mod.rs"]
pub mod storage;
#[path = "../src/capture/mod.rs"]
pub mod capture;
#[path = "../src/timing/mod.rs"]
pub mod timing;
#[cfg(test)]
#[path = "navigation_tests.rs"]
mod navigation_tests;
