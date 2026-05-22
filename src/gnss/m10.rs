use alloc::vec::Vec;

use ublox::cfg_msg::CfgMsgSinglePortBuilder;
use ublox::cfg_prt::{
    CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId,
};
use ublox::cfg_rate::{AlignmentToReferenceTime, CfgRateBuilder};
use ublox::mon_rf::MonRf;
use ublox::mon_ver::MonVer;
use ublox::nav_pvt::proto27::NavPvt;
use ublox::nav_sat::NavSat;
use ublox::proto27::{PacketRef, Proto27};
use ublox::{GnssFixType, Parser, PositionLLA, UbxPacket, UbxPacketRequest, Velocity};

use super::{Fix, GnssReceiver, GnssUpdate};

const BAUD: u32 = 38400;

pub struct UbloxReceiver {
    parser: Parser<Vec<u8>, Proto27>,
}

impl UbloxReceiver {
    pub fn new() -> Self {
        Self { parser: Parser::new(Vec::new()) }
    }
}

impl Default for UbloxReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl GnssReceiver for UbloxReceiver {
    fn baudrate(&self) -> u32 {
        BAUD
    }

    fn init_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();

        out.extend_from_slice(
            &CfgPrtUartBuilder {
                portid: UartPortId::Uart1,
                reserved0: 0,
                tx_ready: 0,
                mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                baud_rate: BAUD,
                in_proto_mask: InProtoMask::UBLOX,
                out_proto_mask: OutProtoMask::UBLOX,
                flags: 0,
                reserved5: 0,
            }
            .into_packet_bytes(),
        );

        out.extend_from_slice(
            &CfgRateBuilder {
                measure_rate_ms: 50,
                nav_rate: 1,
                time_ref: AlignmentToReferenceTime::Gps,
            }
            .into_packet_bytes(),
        );

        out.extend_from_slice(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes());
        out.extend_from_slice(&CfgMsgSinglePortBuilder::set_rate_for::<NavPvt>(1).into_packet_bytes());
        out.extend_from_slice(&CfgMsgSinglePortBuilder::set_rate_for::<NavSat>(20).into_packet_bytes());
        out.extend_from_slice(&CfgMsgSinglePortBuilder::set_rate_for::<MonRf>(50).into_packet_bytes());

        out
    }

    fn consume(&mut self, bytes: &[u8], cb: &mut dyn FnMut(GnssUpdate)) {
        let mut it = self.parser.consume_ubx(bytes);
        while let Some(Ok(UbxPacket::Proto27(pkt))) = it.next() {
            match pkt {
                PacketRef::NavPvt(pvt) => {
                    let has_fix = pvt.fix_type() == GnssFixType::Fix3D
                        || pvt.fix_type() == GnssFixType::GPSPlusDeadReckoning;
                    let pos: PositionLLA = (&pvt).into();
                    let vel: Velocity = (&pvt).into();
                    cb(GnssUpdate::Fix(Fix {
                        lat: pos.lat,
                        lon: pos.lon,
                        alt_m: pos.alt,
                        speed_mps: vel.speed as f32,
                        heading_deg: vel.heading as f32,
                        has_fix,
                        ..Default::default()
                    }));
                }
                PacketRef::NavSat(sat) => {
                    let mut count: u8 = 0;
                    let mut best: u8 = 0;
                    for sv in sat.svs() {
                        count = count.saturating_add(1);
                        if sv.cno() > best {
                            best = sv.cno();
                        }
                    }
                    cb(GnssUpdate::Sats { count, best_cno: best });
                }
                _ => {}
            }
        }
    }
}
