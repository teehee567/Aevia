//! CPU1 adapter for Traj's bounded driving estimator.
mod initialization;
mod measurements;
pub mod memory;
pub(crate) mod race;
pub mod settings;
use crate::display::{Motion, MotionSource};
use aevia_trajectory::{
    engine::driving::{DrivingEstimator, DrivingPoint},
    trajectory::driving::DrivingRun,
};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use heapless::Deque;
use initialization::GnssClock;
use serde::Serialize;

const RUNS_ESTIMATOR: bool = cfg!(all(
    feature = "development-navigation",
    not(feature = "diagnostic-no-navigation")
));
#[derive(Clone, Copy, Serialize)]
pub struct DragResult {
    pub target: u32,
    pub seconds: f64,
    pub stage: &'static str,
    pub validity: &'static str,
}
#[derive(Clone, Copy, Serialize)]
pub struct Status {
    pub phase: &'static str,
    pub qualified: bool,
    pub metrics: &'static str,
    pub timing: &'static str,
    pub error: Option<&'static str>,
    pub updated_us: u64,
    #[serde(skip)]
    processed_imu_us: u64,
    pub epoch_us: Option<u64>,
    pub speed_mps: Option<f64>,
    pub motion: Option<Motion>,
    pub motion_source: MotionSource,
    #[serde(skip)]
    run_start_us: Option<u64>,
    pub driving: crate::display::Timing,
    pub position_ecef_m: Option<[f64; 3]>,
    pub imu_accepted: u64,
    pub imu_bridged: u64,
    pub imu_recovered: u64,
    pub alignment_lost: u64,
    pub gnss_fused: u64,
    pub rejected: u64,
    pub queue_dropped: u32,
    pub maximum_step_us: u32,
    pub errors: u32,
    pub drag: [Option<DragResult>; 2],
}
const INITIAL: Status = Status {
    phase: "starting",
    qualified: false,
    metrics: "preliminary-driving",
    timing: "unqualified",
    error: None,
    updated_us: 0,
    processed_imu_us: 0,
    epoch_us: None,
    speed_mps: None,
    motion: None,
    motion_source: MotionSource::Waiting,
    run_start_us: None,
    driving: crate::display::Timing::EMPTY,
    position_ecef_m: None,
    imu_accepted: 0,
    imu_bridged: 0,
    imu_recovered: 0,
    alignment_lost: 0,
    gnss_fused: 0,
    rejected: 0,
    queue_dropped: 0,
    maximum_step_us: 0,
    errors: 0,
    drag: [None; 2],
};
struct Shared {
    imu: Deque<crate::imu::ImuSample, 256>,
    gnss: Deque<(crate::gnss::OwnedBestNav, u64, Option<u64>), 32>,
    status: Status,
}
static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> = Mutex::new(RefCell::new(Shared {
    imu: Deque::new(),
    gnss: Deque::new(),
    status: INITIAL,
}));
pub fn published_us() -> u64 {
    SHARED.lock(|cell| cell.borrow().status.updated_us)
}
pub fn awaiting_stop(cutoff_us: u64) -> bool {
    RUNS_ESTIMATOR && SHARED.lock(|cell| cell.borrow().status.processed_imu_us < cutoff_us)
}
pub fn snapshot() -> Status {
    let start = crate::app::recording_window().0;
    SHARED.lock(|cell| {
        let mut s = cell.borrow().status;
        if start != s.run_start_us {
            s.driving = crate::display::Timing::EMPTY;
        }
        let now = embassy_time::Instant::now().as_micros();
        let max_age = if s.motion_source == MotionSource::Held {
            500_000
        } else {
            1_000_000
        };
        if s.epoch_us
            .is_some_and(|epoch| now.saturating_sub(epoch) >= max_age)
        {
            s.speed_mps = None;
            s.motion = None;
            s.motion_source = MotionSource::Waiting;
            s.position_ecef_m = None;
            s.phase = "stale";
        }
        s
    })
}
pub fn capture_imu(sample: crate::imu::ImuSample) {
    if !RUNS_ESTIMATOR {
        return;
    }
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.imu.push_back(sample).is_err() {
            s.status.queue_dropped = s.status.queue_dropped.saturating_add(1);
        }
    });
}
pub fn capture_gnss(sample: crate::gnss::OwnedBestNav, received_us: u64, mapped_us: Option<u64>) {
    if !RUNS_ESTIMATOR {
        return;
    }
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.gnss.push_back((sample, received_us, mapped_us)).is_err() {
            s.status.queue_dropped = s.status.queue_dropped.saturating_add(1);
        }
    });
}
fn update(f: impl FnOnce(&mut Status)) {
    SHARED.lock(|s| f(&mut s.borrow_mut().status));
}

struct Processor {
    filter: DrivingEstimator,
    run: DrivingRun,
    previous: Option<DrivingPoint>,
    run_start: Option<u64>,
    clock: GnssClock,
    last_publish: u64,
    processed_imu_us: u64,
    published_stop: Option<u64>,
    imu_count: u64,
    fix_count: u64,
    rejected: u64,
}
static mut PROCESSOR: Option<Processor> = None;
fn estimator() -> DrivingEstimator {
    DrivingEstimator::new(settings::BODY_FROM_IMU_WXYZ.map(|x| x as f32))
        .expect("valid IMU installation")
}

pub fn initialize(psram: esp_hal::psram::Psram) {
    if esp_hal::system::Cpu::current() != esp_hal::system::Cpu::AppCpu {
        return;
    }
    // The USB raw-preview buffer owns its reserved external-memory region.
    let _ = memory::bring_up(psram);
    if cfg!(feature = "diagnostic-no-navigation") {
        update(|s| s.phase = "disabled");
        return;
    }
    if settings::IMU_TO_ANTENNA_METRES != [0.; 3] {
        update(|s| {
            s.phase = "failed";
            s.error = Some("driving-estimator-requires-colocated-antenna");
        });
        return;
    }
    if !cfg!(feature = "development-navigation") {
        update(|s| {
            s.phase = "failed";
            s.error = Some("production-profile-unqualified");
        });
        return;
    }
    unsafe {
        *core::ptr::addr_of_mut!(PROCESSOR) = Some(Processor {
            filter: estimator(),
            run: race::snapshot().run(),
            previous: None,
            run_start: None,
            clock: GnssClock::default(),
            last_publish: 0,
            processed_imu_us: 0,
            published_stop: None,
            imu_count: 0,
            fix_count: 0,
            rejected: 0,
        });
    }
    update(|s| s.phase = "aligning-at-rest");
}

impl Processor {
    fn fix(&mut self, g: crate::gnss::BestNav<'_>, received: u64, mapped_us: Option<u64>) {
        let Some(epoch) = crate::timing::gps_epoch_ms(g.gps_week, g.gps_tow_ms) else {
            return;
        };
        let Some(time) = self.clock.observe(epoch, received, mapped_us) else {
            return;
        };
        if time.discontinuity {
            self.filter.reset_gnss_clock();
            self.previous = None;
        }
        update(|s| s.timing = time.source);
        if self
            .filter
            .observe_fix(measurements::prepare_gnss(g, time.time_us))
        {
            self.fix_count += 1;
        } else {
            self.rejected += 1;
        }
    }

    fn imu(&mut self, i: crate::imu::ImuSample) {
        let (start, stop) = crate::app::recording_window();
        if start != self.run_start {
            self.run = race::snapshot().run();
            self.run_start = start;
        }
        if !self.filter.observe_imu(
            i.effective_end_ns / 1000,
            i.interval_ns / 1000,
            i.acceleration_mps2.map(|x| x as f32),
            i.aligned_angular_rate_rps.map(|x| x as f32),
        ) {
            self.rejected += 1;
            return;
        }
        self.imu_count += 1;
        let current = self.filter.point();
        // Keep the filter warm after Stop, and freeze the completed lap and
        // sector state once the interval containing the cutoff was processed.
        if let Some(start) = start
            && stop.is_none_or(|cutoff| self.processed_imu_us < cutoff)
        {
            if let (Some(a), Some(b)) = (self.previous, current) {
                self.run
                    .advance_points(a, b, start, stop, i.interval_ns / 1000);
            } else {
                self.run.interrupt();
            }
        }
        self.processed_imu_us = i.effective_end_ns / 1000;
        self.previous = current;
    }

    fn publish(&self, now: u64) {
        let motion = self
            .filter
            .motion(race::mounted())
            .filter(|m| now.saturating_sub(m.point.epoch_us) < 250_000);
        let gnss = if motion.is_none() {
            crate::app::latest_gnss_preview(now)
        } else {
            None
        };
        let r = self.run.totals;
        let timing = crate::display::Timing {
            distance_m: r.distance_m as f32,
            average_mps: (r.duration_s > 0.).then(|| (r.distance_m / r.duration_s) as f32),
            peak_mps: (r.duration_s > 0.).then_some(r.peak_mps as f32),
            laps: r.laps,
            lap_start_us: r.lap_start_s.map(|s| (s * 1e6) as u64),
            last_lap_end_us: r.last_lap_end_s.map(|s| (s * 1e6) as u64),
            last_lap_s: r.last_lap_s.map(|s| s as f32),
            best_lap_s: r.best_lap_s.map(|s| s as f32),
            sector: r.sector,
            last_sector_s: r.last_sector_s.map(|s| s as f32),
            timing_origin_us: r.timing_origin_s.map(|s| (s * 1e6) as u64),
            launch_us: r.launch_s.map(|s| (s * 1e6) as u64),
            speed_time_s: r.speed_time_s.map(|s| s as f32),
            distance_time_s: r.distance_time_s.map(|s| s as f32),
        };

        update(|s| {
            let (source, preview) = if let Some(m) = motion {
                let source = if m.inertial_only {
                    MotionSource::Inertial
                } else if m.position_degraded {
                    MotionSource::PositionDegraded
                } else {
                    MotionSource::Fused
                };
                (
                    source,
                    Some(Motion {
                        t_us: m.point.epoch_us,
                        lat: m.latitude_deg,
                        lon: m.longitude_deg,
                        alt_m: m.height_m as f32,
                        speed_mps: m.point.speed_mps,
                        course_deg: m.course_deg,
                        heading_deg: m.heading_deg,
                        vehicle_g: m.vehicle_g,
                        degraded: source != MotionSource::Fused,
                    }),
                )
            } else if let Some(gnss) = gnss {
                (MotionSource::Gnss, Some(gnss))
            } else if let Some(mut last) = s.motion.filter(|m| now.saturating_sub(m.t_us) < 500_000)
            {
                last.degraded = true;
                last.vehicle_g = None;
                last.heading_deg = None;
                last.course_deg = None;
                (MotionSource::Held, Some(last))
            } else {
                (MotionSource::Waiting, None)
            };
            s.phase = match source {
                MotionSource::Fused => "driving-complementary",
                MotionSource::PositionDegraded => "driving-doppler-position-degraded",
                MotionSource::Inertial => "inertial-coast",
                MotionSource::Gnss => "gnss-preview",
                MotionSource::Held => "holding-last-motion",
                MotionSource::Waiting => "waiting-for-alignment-or-fix",
            };
            s.updated_us = now;
            s.processed_imu_us = self.processed_imu_us;
            s.epoch_us = preview.map(|m| m.t_us);
            s.speed_mps = preview.map(|m| f64::from(m.speed_mps));
            s.position_ecef_m = motion.map(|m| m.point.position_ecef_m);
            s.motion = preview;
            s.motion_source = source;
            s.driving = timing;
            s.run_start_us = self.run_start;
            s.imu_accepted = self.imu_count;
            s.imu_bridged = self.filter.bridged_intervals;
            s.imu_recovered = self.filter.recovered_imu_gaps;
            s.alignment_lost = self.filter.lost_alignment;
            s.gnss_fused = self.fix_count;
            s.rejected = self.rejected;
        });
    }
}

pub fn tick() {
    if !RUNS_ESTIMATOR {
        return;
    }
    let Some(p) = (unsafe { &mut *core::ptr::addr_of_mut!(PROCESSOR) }).as_mut() else {
        return;
    };
    let begin = embassy_time::Instant::now();
    // Service observations in receipt order; check the card between bounded batches.
    for _ in 0..8 {
        let (imu, gnss) = SHARED.lock(|cell| {
            let mut s = cell.borrow_mut();
            if s.gnss
                .front()
                .is_some_and(|(_, t, _)| s.imu.front().is_none_or(|i| *t <= i.monotonic_us))
            {
                (None, s.gnss.pop_front())
            } else {
                (s.imu.pop_front(), None)
            }
        });
        if let Some(i) = imu {
            p.imu(i);
        } else if let Some((g, t, mapped)) = gnss {
            p.fix(g.as_nav(), t, mapped);
        } else {
            break;
        }
        if begin.elapsed().as_micros() >= 1500 {
            break;
        }
    }
    let now = embassy_time::Instant::now().as_micros();
    let stop = crate::app::recording_window()
        .1
        .filter(|t| *t <= p.processed_imu_us);
    if now.saturating_sub(p.last_publish) >= 200_000 || (stop.is_some() && stop != p.published_stop)
    {
        p.published_stop = stop;
        p.publish(now);
        p.last_publish = now;
    }
    update(|s| s.maximum_step_us = s.maximum_step_us.max(begin.elapsed().as_micros() as u32));
}
