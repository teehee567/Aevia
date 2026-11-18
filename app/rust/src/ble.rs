use btleplug::{
    api::{Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType},
    platform::{Adapter, Manager, Peripheral},
};
use std::time::Duration;
use tokio::{
    runtime::Runtime,
    time::{sleep, timeout},
};
use uuid::Uuid;

const LIVE: Uuid = Uuid::from_u128(0xcf2c0001_183e_4cd6_a712_c719ac51bd52);
const FRAME: Uuid = Uuid::from_u128(0xcf2c0002_183e_4cd6_a712_c719ac51bd52);
const RECORDING: Uuid = Uuid::from_u128(0xcf2c0003_183e_4cd6_a712_c719ac51bd52);

pub struct Connection {
    device: Peripheral,
    frame: Characteristic,
    recording: Characteristic,
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
        let (adapter, device, frame, recording) = runtime.block_on(async {
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
            _adapter: adapter,
        })
    }

    pub fn read(&self) -> Result<Vec<u8>, String> {
        self.runtime.block_on(async {
            timeout(Duration::from_secs(3), self.device.read(&self.frame))
                .await
                .map_err(|_| "Bluetooth display read timed out")?
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

async fn discover(
) -> Result<(Adapter, Peripheral, Characteristic, Characteristic), Box<dyn std::error::Error>> {
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
        Ok::<_, Box<dyn std::error::Error>>((get(FRAME)?, get(RECORDING)?))
    }
    .await;
    if result.is_err() {
        let _ = device.disconnect().await;
    }
    let (frame, recording) = result?;
    Ok((adapter, device, frame, recording))
}
