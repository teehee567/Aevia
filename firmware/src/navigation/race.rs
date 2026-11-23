use crate::display::{Configure, MapGate};
use aevia_trajectory::trajectory::driving::{DrivingGate, DrivingRun};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

#[derive(Clone, Copy)]
pub(super) struct RaceConfig {
    start: u32,
    end: u32,
    distance: u32,
    rollout: bool,
    pub mounted: bool,
    expected: u8,
    received: u8,
    gates: [MapGate; 16],
}
const DEFAULT: RaceConfig = RaceConfig {
    start: 0,
    end: 10000,
    distance: 402336,
    rollout: false,
    mounted: false,
    expected: 0,
    received: 0,
    gates: [MapGate {
        lat_e7: 0,
        lon_e7: 0,
        heading_cdeg: 0,
        width_cm: 0,
    }; 16],
};
struct Settings {
    committed: RaceConfig,
    pending: Option<RaceConfig>,
}
static CONFIG: Mutex<CriticalSectionRawMutex, RefCell<Settings>> =
    Mutex::new(RefCell::new(Settings {
        committed: DEFAULT,
        pending: None,
    }));

pub(crate) fn configure(command: Configure) -> Result<(), ()> {
    CONFIG.lock(|cell| {
        let mut settings = cell.borrow_mut();
        match command {
            Configure::Mount { confirmed } => {
                settings.pending = None;
                settings.committed.mounted = confirmed;
            }
            Configure::Begin {
                start_centi_kph: start,
                end_centi_kph: end,
                distance_mm: distance,
                rollout,
                mounted,
                gates,
            } if gates <= 16
                && end > start
                && end <= 100000
                && (1000..=10000000).contains(&distance) =>
            {
                settings.pending = Some(RaceConfig {
                    start,
                    end,
                    distance,
                    rollout,
                    mounted,
                    expected: gates,
                    ..DEFAULT
                });
            }
            Configure::Gate { index, gate } => {
                let config = settings.pending.as_mut().ok_or(())?;
                if index != config.received
                    || index >= config.expected
                    || !(-900000000..=900000000).contains(&gate.lat_e7)
                    || !(-1800000000..=1800000000).contains(&gate.lon_e7)
                    || gate.heading_cdeg >= 36000
                    || !(100..=50000).contains(&gate.width_cm)
                {
                    return Err(());
                }
                config.gates[usize::from(index)] = gate;
                config.received += 1;
            }
            _ => return Err(()),
        }
        if settings
            .pending
            .as_ref()
            .is_some_and(|p| p.expected == p.received)
        {
            settings.committed = settings.pending.take().unwrap();
        }
        Ok(())
    })
}
pub(super) fn snapshot() -> RaceConfig {
    CONFIG.lock(|c| c.borrow().committed)
}
/// Serialize the committed race setup into each independently importable SD part.
pub(crate) fn visit_configuration<E>(
    mut visit: impl FnMut(Configure) -> Result<(), E>,
) -> Result<(), E> {
    let config = snapshot();
    visit(Configure::Begin {
        start_centi_kph: config.start,
        end_centi_kph: config.end,
        distance_mm: config.distance,
        rollout: config.rollout,
        mounted: config.mounted,
        gates: config.received,
    })?;
    for index in 0..config.received {
        visit(Configure::Gate {
            index,
            gate: config.gates[usize::from(index)],
        })?;
    }
    Ok(())
}
pub(crate) fn mounted() -> bool {
    CONFIG.lock(|c| c.borrow().committed.mounted)
}
impl RaceConfig {
    pub(super) fn run(self) -> DrivingRun {
        let mut run = DrivingRun::new(
            self.start as f64 / 360.,
            self.end as f64 / 360.,
            self.distance as f64 / 1000.,
            self.rollout,
        );
        for g in &self.gates[..usize::from(self.received)] {
            let gate = DrivingGate::from_wgs84(
                g.lat_e7 as f64 / 1e7,
                g.lon_e7 as f64 / 1e7,
                g.heading_cdeg as f64 / 100.,
                g.width_cm as f64 / 100.,
            )
            .unwrap();
            run.add_gate(gate).unwrap();
        }
        run
    }
}
