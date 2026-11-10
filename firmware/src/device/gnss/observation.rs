//! Portable GNSS-to-estimator boundary, shared by the worker and host regressions.
use aevia_trajectory::{
    StepError,
    frame::{EcefPosition, EcefVelocity},
    ids::{ClockModelId, FrameId, ObservationId, ReferencePointId, SourceId},
    observation::{
        GnssDiagnostics, GnssPosition, GnssSolutionObservation, GnssVelocity, ReceiverHealth,
        TimedDiagnostic,
    },
    time::{
        DurationNs, ObservationTime, SampleSupport, SessionTime, SignedDurationNs, TimingBasis,
    },
    uncertainty::{Covariance3, MeasurementUncertainty},
};

#[derive(Clone, Copy, Debug)]
#[cfg_attr(test, allow(dead_code))]
pub struct GnssSample {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    pub position_sigma_m: [f64; 3],
    pub velocity_sigma_mps: [f64; 3],
    pub epoch_ms: u64,
    pub received_us: u64,
    pub position_time_ns: u64,
    pub velocity_time_ns: u64,
    pub timing_uncertainty_ns: u64,
    pub clock_generation: u32,
    pub position_valid: bool,
    pub velocity_valid: bool,
    pub satellites: u8,
}

pub(crate) fn observation(
    sample: GnssSample,
    sequence: u64,
    frame: FrameId,
) -> Result<GnssSolutionObservation, StepError> {
    let time = observation_time(sample.position_time_ns, sample.timing_uncertainty_ns)?;
    let velocity_time = observation_time(sample.velocity_time_ns, sample.timing_uncertainty_ns)?;
    // A solution's status describes its valid fields. One missing companion is
    // not evidence that the surviving measurement is unhealthy. The earliest
    // valid support epoch makes this evidence available to each timed member.
    let health_time = if sample.velocity_valid
        && (!sample.position_valid || sample.velocity_time_ns < sample.position_time_ns)
    {
        velocity_time
    } else {
        time
    };
    GnssSolutionObservation::new(
        ObservationId::new(SourceId::new(2), sequence),
        ReferencePointId::new(2),
        Some(GnssPosition {
            value: EcefPosition::from_components(sample.position_m)
                .map_err(StepError::InvalidObservation)?,
            time,
            frame,
            valid: sample.position_valid,
            uncertainty: MeasurementUncertainty::Provided(diagonal(
                sample.position_sigma_m.map(|v| v.max(0.02)),
            )?),
        }),
        Some(GnssVelocity {
            value: EcefVelocity::from_components(sample.velocity_mps)
                .map_err(StepError::InvalidObservation)?,
            time: velocity_time,
            frame,
            valid: sample.velocity_valid,
            uncertainty: MeasurementUncertainty::Provided(diagonal(
                sample.velocity_sigma_mps.map(|v| v.max(0.03)),
            )?),
        }),
        None,
        GnssDiagnostics {
            // Navigation::prepare already enforces receiver ages per field.
            // Position solution/correction age is not the age of independent
            // Doppler velocity, so it must not be attached as shared diagnostics.
            correction_age: None,
            solution_age: None,
            health: Some(TimedDiagnostic {
                value: if sample.position_valid || sample.velocity_valid {
                    ReceiverHealth::Healthy
                } else {
                    ReceiverHealth::Suspect
                },
                time: health_time,
                age: DurationNs::ZERO,
            }),
        },
    )
    .map_err(StepError::InvalidObservation)
}

/// Initialization needs a valid position/velocity pair accepted by the engine.
/// A partial solution or a retained/rejected observation cannot open IMU work.
pub(crate) fn initialization_seed_accepted(
    observation: GnssSolutionObservation,
    disposition: aevia_trajectory::observation::InputDisposition,
) -> bool {
    use aevia_trajectory::observation::InputDisposition;
    observation.position().is_some_and(|field| field.valid)
        && observation.velocity().is_some_and(|field| field.valid)
        && matches!(
            disposition,
            InputDisposition::InitializationOnly
                | InputDisposition::QueuedForFusion
                | InputDisposition::Fused
                | InputDisposition::Downweighted
        )
}

fn observation_time(nanos: u64, sigma_ns: u64) -> Result<ObservationTime, StepError> {
    let nanos = i64::try_from(nanos).map_err(|_| StepError::ClockDiscontinuity)?;
    Ok(ObservationTime {
        registered_at: SessionTime::from_ns(nanos),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::from_ns(sigma_ns),
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis: TimingBasis::PpsCorrelated,
    })
}

fn diagonal(sigma: [f64; 3]) -> Result<Covariance3, StepError> {
    Covariance3::diagonal(
        sigma[0] * sigma[0],
        sigma[1] * sigma[1],
        sigma[2] * sigma[2],
    )
    .map_err(StepError::InvalidObservation)
}

#[cfg(test)]
mod tests;
