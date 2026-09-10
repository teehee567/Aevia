//! TCA9536A button inputs, owned by the power-bus task.
//! Register map: https://www.ti.com/lit/ds/symlink/tca9536.pdf

use aevia_firmware::buttons::Debouncer;
use core::cell::Cell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use esp_hal::{Async, i2c::master::I2c};

const ADDRESS: u8 = 0x40;
static STATUS: Mutex<CriticalSectionRawMutex, Cell<Option<u8>>> = Mutex::new(Cell::new(None));

pub fn pressed() -> Option<u8> {
    STATUS.lock(Cell::get)
}

#[derive(Default)]
pub struct Buttons {
    debounce: Debouncer,
    ready: bool,
    retry_ms: u64,
}

impl Buttons {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn poll(&mut self, bus: &mut I2c<'_, Async>, now_ms: u64) {
        if now_ms < self.retry_ms {
            return;
        }
        let result = async {
            if !self.ready {
                // All four pins are buttons. Never drive them as outputs.
                bus.write_async(ADDRESS, &[0x03, 0xff]).await?;
                bus.write_async(ADDRESS, &[0x02, 0x00]).await?;
                bus.write_async(ADDRESS, &[0x50, 0x00]).await?; // Inputs with internal pull-ups.
                let mut config = [0];
                bus.write_read_async(ADDRESS, &[0x03], &mut config).await?;
                if config[0] & 0x0f != 0x0f {
                    return Ok(None);
                }
                self.ready = true;
            }
            let mut input = [0];
            bus.write_read_async(ADDRESS, &[0], &mut input).await?;
            Ok::<_, esp_hal::i2c::master::Error>(Some(input[0]))
        }
        .await;
        let input = result.ok().flatten();
        if input.is_none() {
            self.ready = false;
            self.retry_ms = now_ms + 1_000;
        }
        let stable = self.debounce.update(now_ms, input);
        STATUS.lock(|state| state.set(stable));
    }
}
