use embassy_nrf::buffered_uarte::{self, BufferedUarte};
use embassy_nrf::gpio::Pin;
use embassy_nrf::peripherals::{PPI_CH3, PPI_CH4, PPI_GROUP1, TIMER1, UARTE0};
use embassy_nrf::uarte::{self, Baudrate, Config};
use embassy_nrf::{bind_interrupts, Peri};
use embedded_io_async::{Read, Write};

use race_core::gnss::{GnssReceiver, GnssUpdate};

bind_interrupts!(struct Irqs {
    UARTE0 => buffered_uarte::InterruptHandler<UARTE0>;
});

fn baudrate(hz: u32) -> Baudrate {
    match hz {
        9600 => Baudrate::BAUD9600,
        38400 => Baudrate::BAUD38400,
        115200 => Baudrate::BAUD115200,
        921600 => Baudrate::BAUD921600,
        _ => Baudrate::BAUD38400,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn uart<'d>(
    uarte: Peri<'d, UARTE0>,
    timer: Peri<'d, TIMER1>,
    ppi_ch1: Peri<'d, PPI_CH3>,
    ppi_ch2: Peri<'d, PPI_CH4>,
    ppi_group: Peri<'d, PPI_GROUP1>,
    rxd: Peri<'d, impl Pin>,
    txd: Peri<'d, impl Pin>,
    baud: u32,
) -> BufferedUarte<'d> {
    let mut config = Config::default();
    config.baudrate = baudrate(baud);
    config.parity = uarte::Parity::EXCLUDED;

    static mut RX_BUF: [u8; 1024] = [0; 1024];
    static mut TX_BUF: [u8; 1024] = [0; 1024];

    BufferedUarte::new(
        uarte,
        timer,
        ppi_ch1,
        ppi_ch2,
        ppi_group,
        rxd,
        txd,
        Irqs,
        config,
        unsafe { &mut RX_BUF },
        unsafe { &mut TX_BUF },
    )
}

pub struct Gps<C, R> {
    comm: C,
    receiver: R,
}

impl<C: Read + Write, R: GnssReceiver> Gps<C, R> {
    pub fn new(comm: C, receiver: R) -> Self {
        Self { comm, receiver }
    }

    pub async fn configure(&mut self) {
        let init = self.receiver.init_bytes();
        let _ = self.comm.write_all(&init).await;
    }

    pub async fn poll<F: FnMut(GnssUpdate)>(&mut self, mut on_update: F) {
        let mut chunk = [0u8; 64];
        if let Ok(n) = self.comm.read(&mut chunk).await {
            if n > 0 {
                let now = embassy_time::Instant::now();
                crate::status::update(|s| s.last_uart = Some(now));
            }
            self.receiver.consume(&chunk[..n], &mut on_update);
        }
    }
}
