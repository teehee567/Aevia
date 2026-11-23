use btleplug::{
    api::{Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType},
    platform::{Adapter, Manager, Peripheral},
};
use std::time::{Duration, Instant};
use tokio::{
    runtime::Runtime,
    time::{sleep, timeout},
};
use uuid::Uuid;

const LIVE: Uuid = Uuid::from_u128(0xcf2c0001_183e_4cd6_a712_c719ac51bd52);
const FRAME: Uuid = Uuid::from_u128(0xcf2c0004_183e_4cd6_a712_c719ac51bd52);
const SETTINGS: Uuid = Uuid::from_u128(0xcf2c0005_183e_4cd6_a712_c719ac51bd52);
const RECORDING: Uuid = Uuid::from_u128(0xcf2c0003_183e_4cd6_a712_c719ac51bd52);

pub struct Connection {
    device: Peripheral,
    frame: Characteristic,
    recording: Characteristic,
    settings: Characteristic,
    detail_version: u8,
    battery_sample: Option<(Instant, serde_json::Value)>,
    _adapter: Adapter,
    runtime: Runtime,
}

impl Connection {
    pub fn connect() -> Result<Self, String> {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .map_err(|e| e.to_string())?;
        let (adapter, device, frame, recording, settings, detail_version) =
            runtime.block_on(async {
                timeout(Duration::from_secs(45), discover())
                    .await
                    .map_err(|_| "Bluetooth timed out. Check Bluetooth permissions and try again.")?
                    .map_err(|e| e.to_string())
            })?;
        Ok(Self {
            runtime,
            device,
            frame,
            recording,
            settings,
            detail_version,
            battery_sample: None,
            _adapter: adapter,
        })
    }

    pub fn read(&mut self) -> Result<serde_json::Value, String> {
        let bytes = self.runtime.block_on(async {
            timeout(Duration::from_secs(3), self.device.read(&self.frame))
                .await
                .map_err(|_| "Bluetooth display read timed out")?
                .map_err(|e| e.to_string())
        })?;
        if self.detail_version == 0 {
            return crate::driving::decode(&bytes);
        }
        let (mut reply, sample) =
            crate::driving::decode_battery_display(&bytes, self.detail_version)?;
        if let Some(sample) = sample {
            self.battery_sample = Some((Instant::now(), sample));
        }
        if let Some((at, battery)) = &self.battery_sample {
            reply["battery"] = battery.clone();
            reply["battery_age_ms"] = serde_json::json!(at.elapsed().as_millis() as u64);
        } else {
            reply["battery"] = serde_json::Value::Null;
            reply["battery_age_ms"] = serde_json::Value::Null;
        }
        Ok(reply)
    }

    pub fn configure(&self, command: &crate::display::Configure) -> Result<(), String> {
        let mut buf = [0; 32];
        let data = postcard::to_slice(command, &mut buf).map_err(|e| e.to_string())?;
        self.runtime.block_on(async {
            timeout(
                Duration::from_secs(3),
                self.device
                    .write(&self.settings, data, WriteType::WithResponse),
            )
            .await
            .map_err(|_| "Bluetooth configuration timed out")?
            .map_err(|e| e.to_string())
        })
    }
    pub fn record(&self, active: bool) -> Result<(), String> {
        self.runtime.block_on(async {
            timeout(
                Duration::from_secs(12),
                self.device.write(
                    &self.recording,
                    &[u8::from(active)],
                    WriteType::WithResponse,
                ),
            )
            .await
            .map_err(|_| "Receiver did not confirm recording control")?
            .map_err(|e| e.to_string())
        })
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        self.runtime.block_on(async {
            let _ = timeout(Duration::from_secs(2), self.device.disconnect()).await;
        });
    }
}

// Windows can retain a just-closed GATT link for several seconds before the
// receiver is able to advertise again.
async fn find_receiver(adapter: &Adapter) -> Result<Peripheral, Box<dyn std::error::Error>> {
    for _ in 0..20 {
        sleep(Duration::from_millis(1500)).await;
        let mut found = None;
        for device in adapter.peripherals().await? {
            if device
                .properties()
                .await?
                .is_some_and(|p| p.services.contains(&LIVE))
            {
                if found.is_some() {
                    return Err(
                        "Multiple V2 Minis nearby. Power on only the receiver you want to connect."
                            .into(),
                    );
                }
                found = Some(device);
            }
        }
        if let Some(device) = found {
            return Ok(device);
        }
    }
    Err("No V2 Mini found. Check receiver power and Bluetooth permission, then retry.".into())
}

async fn discover() -> Result<
    (
        Adapter,
        Peripheral,
        Characteristic,
        Characteristic,
        Characteristic,
        u8,
    ),
    Box<dyn std::error::Error>,
> {
    let adapter = Manager::new()
        .await?
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("No Bluetooth adapter available")?;
    adapter
        .start_scan(ScanFilter {
            services: vec![LIVE],
        })
        .await?;
    let found = find_receiver(&adapter).await;
    adapter.stop_scan().await?;
    let device = found?;
    device.connect().await?;
    let result = async {
        device.discover_services().await?;
        let chars = device.characteristics();
        let get = |id| {
            chars
                .iter()
                .find(|c| c.service_uuid == LIVE && c.uuid == id)
                .cloned()
                .ok_or("Install the matching V2 Mini Bluetooth firmware")
        };
        let frame = get(FRAME)?;
        let recording = get(RECORDING)?;
        let settings = get(SETTINGS)?;
        // Older peripherals reject the unknown settings command and continue
        // serving their original percentage-only driving frames.
        let mut detail_version = 0;
        for (version, tag) in [
            (4, crate::display::IMU_CLIPPING_TAG),
            (3, crate::display::MOTION_SOURCE_TAG),
            (2, crate::display::THERMAL_TELEMETRY_TAG),
            (1, crate::display::BATTERY_TELEMETRY_TAG),
        ] {
            if timeout(
                Duration::from_secs(3),
                device.write(&settings, tag, WriteType::WithResponse),
            )
            .await
            .map_err(|_| "Bluetooth telemetry negotiation timed out")?
            .is_ok()
            {
                detail_version = version;
                break;
            }
        }
        Ok::<_, Box<dyn std::error::Error>>((frame, recording, settings, detail_version))
    }
    .await;
    if result.is_err() {
        let _ = device.disconnect().await;
    }
    let (frame, recording, settings, detail_version) = result?;
    Ok((adapter, device, frame, recording, settings, detail_version))
}
