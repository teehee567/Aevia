//! Live display reads and acknowledged SD recording controls.
use crate::display::{
    BATTERY_TELEMETRY_TAG, IMU_CLIPPING_BIT, IMU_CLIPPING_TAG, MAX_BLE_FRAME_BYTES,
    MAX_DRIVING_FRAME_BYTES, MOTION_SOURCE_TAG, THERMAL_TELEMETRY_TAG,
};
use embassy_futures::select::select;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::peripherals::BT;
use esp_radio::ble::controller::BleConnector;
use trouble_host::{
    att::{AttClient, AttReq},
    prelude::*,
};

const SERVICE: u128 = 0xcf2c0001_183e_4cd6_a712_c719ac51bd52;
#[gatt_service(uuid = SERVICE)]
struct LiveService {
    #[characteristic(uuid = "cf2c0004-183e-4cd6-a712-c719ac51bd52", read)]
    frame: heapless::Vec<u8, MAX_BLE_FRAME_BYTES>,
    #[characteristic(uuid = "cf2c0003-183e-4cd6-a712-c719ac51bd52", write)]
    recording: u8,
    #[characteristic(uuid = "cf2c0005-183e-4cd6-a712-c719ac51bd52", write)]
    settings: heapless::Vec<u8, 32>,
}

#[gatt_server(connections_max = 1)]
struct Server {
    live: LiveService,
}

fn refresh_frame(
    server: &Server<'_>,
    include_battery: bool,
    detail_version: u8,
) -> Result<(), AttErrorCode> {
    let (frame, source) = crate::app::display_frame(true);
    let mut buf = [0; MAX_DRIVING_FRAME_BYTES];
    let bytes = postcard::to_slice(&frame, &mut buf).map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
    let mut value = heapless::Vec::from_slice(bytes).map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
    if include_battery {
        let mut trailer = [0; 20];
        let bytes = postcard::to_slice(&crate::tasks::power::snapshot().telemetry(), &mut trailer)
            .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
        value
            .extend_from_slice(if detail_version >= 2 {
                THERMAL_TELEMETRY_TAG
            } else {
                BATTERY_TELEMETRY_TAG
            })
            .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
        value
            .extend_from_slice(bytes)
            .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
        if detail_version >= 2 {
            value
                .extend_from_slice(&crate::tasks::thermal::packed())
                .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
        }
    }
    if detail_version >= 3 {
        let clipping = detail_version >= 4 && crate::app::imu_clipping(frame.uptime_us);
        let status = source as u8 | if clipping { IMU_CLIPPING_BIT } else { 0 };
        value
            .push(status)
            .map_err(|_| AttErrorCode::UNLIKELY_ERROR)?;
    }
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
                let mut battery_due = None;
                let mut detail_version = 0;
                // Read Blob continuations belong to this connection.
                let _ = server.set(&server.live.frame, &heapless::Vec::new());
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
                            let include_battery =
                                battery_due.is_some_and(|due| Instant::now() >= due);
                            let reply = match initial
                                .then(|| refresh_frame(&server, include_battery, detail_version))
                            {
                                Some(Err(err)) => read.reject(err),
                                Some(Ok(())) if include_battery => {
                                    battery_due = Some(Instant::now() + Duration::from_secs(5));
                                    read.accept()
                                }
                                _ => read.accept(),
                            };
                            if let Ok(reply) = reply {
                                reply.send().await;
                            }
                        }
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Write(write),
                        } if write.handle() == server.live.settings.handle => {
                            let accepted = match write.value(&server.live.settings) {
                                Ok(bytes) if bytes.as_slice() == BATTERY_TELEMETRY_TAG => {
                                    detail_version = 1;
                                    battery_due.get_or_insert_with(Instant::now);
                                    true
                                }
                                Ok(bytes) if bytes.as_slice() == THERMAL_TELEMETRY_TAG => {
                                    detail_version = 2;
                                    battery_due.get_or_insert_with(Instant::now);
                                    true
                                }
                                Ok(bytes) if bytes.as_slice() == MOTION_SOURCE_TAG => {
                                    detail_version = 3;
                                    battery_due.get_or_insert_with(Instant::now);
                                    true
                                }
                                Ok(bytes) if bytes.as_slice() == IMU_CLIPPING_TAG => {
                                    detail_version = 4;
                                    battery_due.get_or_insert_with(Instant::now);
                                    true
                                }
                                Ok(bytes) => postcard::from_bytes(&bytes).is_ok_and(|command| {
                                    crate::app::configure_drive(command).is_ok()
                                }),
                                Err(_) => false,
                            };
                            let reply = if accepted {
                                write.accept()
                            } else {
                                write.reject(AttErrorCode::VALUE_NOT_ALLOWED)
                            };
                            if let Ok(reply) = reply {
                                reply.send().await;
                            }
                        }
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Write(write),
                        } if write.handle() == server.live.recording.handle => {
                            let result = match write.value(&server.live.recording) {
                                Ok(0) => crate::app::finish_recording()
                                    .await
                                    .map(|_| ())
                                    .map_err(|_| AttErrorCode::UNLIKELY_ERROR),
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
