use crate::gnss::Fix;

pub const fn kmh(v: f32) -> f32 {
    v / 3.6
}

#[derive(Clone, Copy, Debug)]
pub struct RunConfig {
    pub stationary_mps: f32,
    pub launch_mps: f32,
    pub target_mps: f32,
    pub arm_samples: u8,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            stationary_mps: 0.5,
            launch_mps: 0.6,
            target_mps: kmh(60.0),
            arm_samples: 10,
        }
    }
}

// current event
#[derive(Clone, Copy, Debug)]
pub enum RunEvent {
    Armed,
    Launched,
    Finished { time_s: f32, distance_m: f32 },
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
    pub accel_mps2: f32,
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
            accel_mps2: 0.0,
        }
    }

    // correct time for velocity time lag
    fn timestamp(fix: &Fix) -> f64 {
        fix.tow_ms as f64 / 1000.0 - fix.vel_latency_s as f64
    }

    // interpolate between crossing target between samples
    fn cross_time(a: Sample, b: Sample, target: f32) -> f64 {
        let dv = b.v - a.v;
        if dv.abs() < f32::EPSILON {
            return b.t;
        }
        a.t + ((target - a.v) / dv) as f64 * (b.t - a.t)
    }

    pub fn update(&mut self, fix: &Fix) -> Option<RunEvent> {
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
                // interpolate backwartds
                self.start_t = Self::cross_time(prev, cur, 0.0).min(cur.t);
                self.distance_m = 0.0;
                self.state = State::Running;
                return Some(RunEvent::Launched);
            }
            State::Running => {
                if cur.v >= self.cfg.target_mps {
                    let cross = Self::cross_time(prev, cur, self.cfg.target_mps);
                    let dt_cross = (cross - prev.t) as f32;
                    self.distance_m += 0.5 * (prev.v + self.cfg.target_mps) * dt_cross;
                    self.reset();
                    return Some(RunEvent::Finished {
                        time_s: (cross - self.start_t) as f32,
                        distance_m: self.distance_m,
                    });
                }
                self.distance_m += 0.5 * (prev.v + cur.v) * dt as f32;
                if stopped {
                    self.reset();
                    return Some(RunEvent::Aborted);
                }
            }
            _ => {}
        }
        None
    }

    fn reset(&mut self) {
        self.state = State::Disarmed;
        self.stationary_count = 0;
    }
}
