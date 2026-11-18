//! Live display reads and acknowledged SD recording controls.
use embassy_futures::select::select;
use embassy_time::Timer;
use esp_hal::peripherals::BT;
use esp_radio::ble::controller::BleConnector;
use trouble_host::{
    att::{AttClient, AttReq},
    prelude::*,
};

const SERVICE: u128 = 0xcf2c0001_183e_4cd6_a712_c719ac51bd52;
#[gatt_service(uuid = SERVICE)]
struct LiveService {
    #[characteristic(uuid = "cf2c0002-183e-4cd6-a712-c719ac51bd52", read)]
    frame: heapless::Vec<u8, 160>,
    #[characteristic(uuid = "cf2c0003-183e-4cd6-a712-c719ac51bd52", write)]
    recording: u8,
}

#[gatt_server(connections_max = 1)]
struct Server {
    live: LiveService,
}

fn refresh_frame(server: &Server<'_>) -> Result<(), AttErrorCode> {
    let mut buf = [0; 160];
    let bytes = postcard::to_slice(&crate::app::display_frame(), &mut buf)
        .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
    let value = heapless::Vec::from_slice(bytes).map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
    server
        .set(&server.live.frame, &value)
        .map_err(|_| AttErrorCode::UNLIKELY_ERROR)
}

#[embassy_executor::task]
pub async fn run(bt: BT<'static>) {
    let config = esp_radio::ble::Config::default().with_max_connections(1);
    let Ok(connector) = BleConnector::new(bt, config) else {
        return;
    };
    let controller = ExternalController::<_, 1>::new(connector);
    let mut resources = HostResources::<_, DefaultPacketPool, 1, 2>::new();
    let stack = trouble_host::new(controller, &mut resources).build();
    let mut peripheral = stack.peripheral();
    let Ok(server) = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Aevia V2 Mini",
        appearance: &appearance::UNKNOWN,
    })) else {
        return;
    };
    let mut adv = [0; 31];
    let Ok(len) = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[SERVICE.to_le_bytes()]),
        ],
        &mut adv,
    ) else {
        return;
    };
    let mut runner = stack.runner();
    select(runner.run(), async {
        loop {
            let params = Default::default();
            let advert = Advertisement::ConnectableScannableUndirected {
                adv_data: &adv[..len],
                scan_data: b"\x0e\x09Aevia V2 Mini",
            };
            if let Ok(acceptor) = peripheral.advertise(&params, advert).await
                && let Ok(link) = acceptor.accept().await
                && let Ok(conn) = link.with_attribute_server(&server)
            {
                loop {
                    match conn.next().await {
                        GattConnectionEvent::Disconnected { .. } => break,
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Read(read),
                        } if read.handle() == server.live.frame.handle => {
                            // Freeze one snapshot through every Read Blob continuation.
                            let initial = matches!(
                                read.payload().incoming(),
                                AttClient::Request(AttReq::Read { .. })
                            );
                            let reply = match initial.then(|| refresh_frame(&server)) {
                                Some(Err(err)) => read.reject(err),
                                _ => read.accept(),
                            };
                            if let Ok(reply) = reply {
                                reply.send().await;
                            }
                        }
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Write(write),
                        } if write.handle() == server.live.recording.handle => {
                            let result = match write.value(&server.live.recording) {
                                Ok(0) => {
                                    crate::app::stop_recording();
                                    if crate::app::wait_for_sd_close().await {
                                        Ok(())
                                    } else {
                                        Err(AttErrorCode::UNLIKELY_ERROR)
                                    }
                                }
                                Ok(1) => crate::app::start_recording(false)
                                    .await
                                    .map_err(|_| AttErrorCode::UNLIKELY_ERROR),
                                _ => Err(AttErrorCode::VALUE_NOT_ALLOWED),
                            };
                            let reply = match result {
                                Ok(()) => write.accept(),
                                Err(err) => write.reject(err),
                            };
                            if let Ok(reply) = reply {
                                reply.send().await;
                            }
                        }
                        _ => {}
                    }
                }
            }
            Timer::after_millis(200).await;
        }
    })
    .await;
}
