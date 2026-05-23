use crate::gnss::Fix;
use log::warn;
use core::fmt;

pub const fn kmh(v: f32) -> f32 {
    v / 3.6
}

#[derive(Clone, Copy, Debug)]
pub enum Target {
    Speed(f32),
    Distance(f32),
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Target::Speed(v) => write!(f, "{:.0} km/h", v * 3.6),
            Target::Distance(d) => write!(f, "{d:.0} m"),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RunConfig {
    pub stationary_mps: f32,
    pub launch_mps: f32,
    pub targets: &'static [Target],
    pub arm_samples: u8,
}

static DEFAULT_TARGETS: [Target; 7] = [
    Target::Speed(kmh(40.0)),
    Target::Speed(kmh(60.0)),
    Target::Speed(kmh(96.6)), // 0-60 mph
    Target::Speed(kmh(100.0)),
    Target::Distance(18.288), // 60 ft
    Target::Distance(201.168), // 1/8 mile
    Target::Distance(402.336), // 1/4 mile
];

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            stationary_mps: 0.5,
            launch_mps: 0.6,
            targets: &DEFAULT_TARGETS,
            arm_samples: 10,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum RunEvent {
    Armed,
    Launched,
    Finished { target: Target, time_s: f32, distance_m: f32 },
    Aborted,
}

#[derive(Clone, Copy)]
struct Sample {
    t: f64, // seconds
    v: f32, // m/s
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Disarmed,
    Armed,
    Running,
}

pub struct RunDetector {
    cfg: RunConfig,
    state: State,
    stationary_count: u8,
    prev: Option<Sample>,
    start_t: f64,
    distance_m: f32,
    target_idx: usize,
    pub accel_mps2: f32,
    pending: [Option<RunEvent>; 8],
    pending_len: usize,
    slow_ticks: u8,
}

impl RunDetector {
    pub fn new(cfg: RunConfig) -> Self {
        Self {
            cfg,
            state: State::Disarmed,
            stationary_count: 0,
            prev: None,
            start_t: 0.0,
            distance_m: 0.0,
            target_idx: 0,
            accel_mps2: 0.0,
            pending: [None; 8],
            pending_len: 0,
            slow_ticks: 0,
        }
    }

    fn push_pending(&mut self, evt: RunEvent) {
        if self.pending_len < self.pending.len() {
            self.pending[self.pending_len] = Some(evt);
            self.pending_len += 1;
        }
    }

    fn pop_pending(&mut self) -> Option<RunEvent> {
        if self.pending_len == 0 {
            return None;
        }
        let evt = self.pending[0].take();
        for i in 0..self.pending_len - 1 {
            self.pending[i] = self.pending[i + 1].take();
        }
        self.pending_len -= 1;
        evt
    }

    // correct time for velocity time lag
    fn timestamp(fix: &Fix) -> f64 {
        fix.tow_ms as f64 / 1000.0 - fix.vel_latency_s as f64
    }

    // interpolate crossing of a speed threshold between two samples
    fn cross_time(a: Sample, b: Sample, target: f32) -> f64 {
        let dv = b.v - a.v;
        if dv.abs() < f32::EPSILON {
            return b.t;
        }
        a.t + ((target - a.v) / dv) as f64 * (b.t - a.t)
    }

    pub fn update(&mut self, fix: &Fix) -> Option<RunEvent> {
        // drain events buffered from a previous tick before processing new data
        if let Some(evt) = self.pop_pending() {
            return Some(evt);
        }

        if !fix.has_fix {
            return None;
        }
        let cur = Sample {
            t: Self::timestamp(fix),
            v: fix.speed_mps,
        };
        let prev = self.prev.replace(cur)?;
        let dt = cur.t - prev.t;
        if dt <= 0.0 {
            return None;
        }
        self.accel_mps2 = (cur.v - prev.v) / dt as f32;

        // 20 Hz = 50 ms per tick; warn after 3 consecutive slow samples
        const EXPECTED_DT: f64 = 1.0 / 20.0;
        if dt > EXPECTED_DT * 1.5 {
            self.slow_ticks = self.slow_ticks.saturating_add(1);
            if self.slow_ticks == 3 {
                warn!("GNSS rate below 20 Hz (dt = {:.0} ms)", dt * 1000.0);
            }
        } else {
            self.slow_ticks = 0;
        }

        let stopped = cur.v <= self.cfg.stationary_mps;

        match self.state {
            State::Disarmed => {
                self.stationary_count = if stopped {
                    self.stationary_count.saturating_add(1)
                } else {
                    0
                };
                if self.stationary_count >= self.cfg.arm_samples {
                    self.state = State::Armed;
                    return Some(RunEvent::Armed);
                }
            }
            State::Armed if cur.v >= self.cfg.launch_mps => {
                self.start_t = Self::cross_time(prev, cur, 0.0).min(cur.t);
                self.distance_m = 0.0;
                self.target_idx = 0;
                self.state = State::Running;
                return Some(RunEvent::Launched);
            }
            State::Running => {
                // tick_start_dist and tick_delta are fixed for this interval so all
                // crossings within the same tick use consistent interpolation bases
                let tick_start_dist = self.distance_m;
                let tick_delta = 0.5 * (prev.v + cur.v) * dt as f32;

                loop {
                    match self.cfg.targets.get(self.target_idx) {
                        Some(&Target::Speed(v)) => {
                            if cur.v < v {
                                self.distance_m = tick_start_dist + tick_delta;
                                break;
                            }
                            let cross = Self::cross_time(prev, cur, v);
                            let dt_cross = (cross - prev.t) as f32;
                            let dist_at_cross = tick_start_dist + 0.5 * (prev.v + v) * dt_cross;
                            self.push_pending(RunEvent::Finished {
                                target: Target::Speed(v),
                                time_s: (cross - self.start_t) as f32,
                                distance_m: dist_at_cross,
                            });
                            self.target_idx += 1;
                            if self.target_idx >= self.cfg.targets.len() {
                                self.distance_m = dist_at_cross;
                                self.reset();
                                break;
                            }
                        }
                        Some(&Target::Distance(d)) => {
                            if tick_start_dist + tick_delta < d {
                                self.distance_m = tick_start_dist + tick_delta;
                                break;
                            }
                            let frac = (d - tick_start_dist) / tick_delta;
                            let cross = prev.t + frac as f64 * dt;
                            self.push_pending(RunEvent::Finished {
                                target: Target::Distance(d),
                                time_s: (cross - self.start_t) as f32,
                                distance_m: d,
                            });
                            self.distance_m = tick_start_dist + tick_delta;
                            self.target_idx += 1;
                            if self.target_idx >= self.cfg.targets.len() {
                                self.reset();
                                break;
                            }
                        }
                        None => {
                            self.reset();
                            break;
                        }
                    }
                }

                if self.state == State::Running && stopped {
                    self.reset();
                    return Some(RunEvent::Aborted);
                }

                return self.pop_pending();
            }
            _ => {}
        }
        None
    }

    fn reset(&mut self) {
        self.state = State::Disarmed;
        self.stationary_count = 0;
        self.target_idx = 0;
    }
}
