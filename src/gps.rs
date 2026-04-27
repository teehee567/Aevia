use alloc::vec::Vec;
use embassy_nrf::buffered_uarte::{self, BufferedUarte};
use embassy_nrf::gpio::Pin;
use embassy_nrf::peripherals::{PPI_CH3, PPI_CH4, PPI_GROUP1, TIMER1, UARTE0};
use embassy_nrf::uarte::{self, Baudrate, Config};
use embassy_nrf::{bind_interrupts, Peri};
use ublox::UbxPacketRequest;
use ublox::cfg_msg::CfgMsgSinglePortBuilder;
use ublox::cfg_nav5::{CfgNav5Params, NavDynamicModel};
use ublox::cfg_prt::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, StopBits, UartMode, UartPortId, Parity};
use ublox::cfg_rate::{AlignmentToReferenceTime, CfgRateBuilder};
use ublox::mon_rf::MonRf;
use ublox::mon_ver::MonVer;
use ublox::nav_pvt::proto27::NavPvt;
use ublox::nav_sat::NavSat;
use ublox::{FixedLinearBuffer, Parser, proto27::Proto27, UbxPacket};

bind_interrupts!(struct Irqs {
    UARTE0 => buffered_uarte::InterruptHandler<UARTE0>;
});

pub struct Gps<'d> {
    uart: BufferedUarte<'d>,
    parser: Parser<FixedLinearBuffer<'d>, Proto27>,

}

impl<'d> Gps<'d> {
    pub fn new(
        uarte: Peri<'d, UARTE0>,
        timer: Peri<'d, TIMER1>,
        ppi_ch1: Peri<'d, PPI_CH3>,
        ppi_ch2: Peri<'d, PPI_CH4>,
        ppi_group: Peri<'d, PPI_GROUP1>,
        rxd: Peri<'d, impl Pin>,
        txd: Peri<'d, impl Pin>,
    ) -> Self {
        let mut config = Config::default();
        config.baudrate = Baudrate::BAUD38400;
        config.parity = uarte::Parity::EXCLUDED;


        // fix buffers
        // fix buffers
        // fix buffers
        // fix buffers
        // fix buffers
        // fix buffers
        // fix buffers
        // fix buffers
        static mut rx_buf: [u8; 1024] = [0; 1024];
        static mut tx_buf: [u8; 1024] = [0; 1024];

        let uart = BufferedUarte::new(
            uarte, timer, ppi_ch1, ppi_ch2, ppi_group, rxd, txd, Irqs, config, unsafe { &mut rx_buf}, unsafe {&mut tx_buf},
        );

        static mut parse_buf: [u8; 1024] = [0; 1024];

        let parser = Parser::new(FixedLinearBuffer::new(unsafe {&mut parse_buf}));


        Self {
            uart,
            parser,
        }
    }

    pub async fn configure(&mut self) {
        // set to use ubx protocol
        let cfg_prt = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
            baud_rate: 38400,
            in_proto_mask: InProtoMask::UBLOX,
            out_proto_mask: OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        }
        .into_packet_bytes();
        let _ = self.uart.write(&cfg_prt).await;

        // set 25hz
        let cfg_rate = CfgRateBuilder {
            measure_rate_ms: 40,
            nav_rate: 1,
            time_ref: AlignmentToReferenceTime::Gps,
        }
        .into_packet_bytes();
        let _ = self.uart.write(&cfg_rate).await;

        let poll_monver = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
        let _ = self.uart.write(&poll_monver).await;

        // enable NavPvt
        let _ = self.uart
            .write(&CfgMsgSinglePortBuilder::set_rate_for::<NavPvt>(1).into_packet_bytes())
            .await;

        // enable NavSat
        let _ = self.uart
            .write(&CfgMsgSinglePortBuilder::set_rate_for::<NavSat>(25).into_packet_bytes())
            .await;

        // enable Monrf
        let _ = self.uart
            .write(&CfgMsgSinglePortBuilder::set_rate_for::<MonRf>(50).into_packet_bytes())
            .await;
    }

    pub async fn poll<F: FnMut(UbxPacket<'_>)>(&mut self, mut on_packet: F) {
        let mut chunk = [0u8; 64];
        if let Ok(n) = self.uart.read(&mut chunk).await {
            if n > 0 {
                let now = embassy_time::Instant::now();
                crate::status::update(|s| s.last_uart = Some(now));
            }
            let mut it = self.parser.consume_ubx(&chunk[..n]);
            while let Some(Ok(pkt)) = it.next() {
                on_packet(pkt);
            }
        }
    }
}
