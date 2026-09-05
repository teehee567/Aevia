//! Conservative raw-tight graph resource estimates and backend capacity limits.

use super::{
    RawTightPassLimits, RawTightPreflightError, RawTightProblemShape, RawTightResourceEstimate,
    RawTightResourceKind,
};
use crate::config::OfflineResourceLimits;
use crate::raw_tight::RawTightBackendRegistration;

pub(super) fn estimate_resources(
    shape: RawTightProblemShape,
) -> Result<RawTightResourceEstimate, RawTightPreflightError> {
    if shape.imu_samples == 0
        || shape.rover_epochs == 0
        || shape.base_or_correction_epochs == 0
        || shape.raw_signal_samples < shape.rover_epochs
        || shape.pseudorange_samples > shape.raw_signal_samples
        || shape.ambiguity_carrier_phase_samples > shape.raw_signal_samples
        || shape.tdcp_samples > shape.ambiguity_carrier_phase_samples
        || shape.doppler_samples > shape.raw_signal_samples
        || shape.pseudorange_samples == 0
        || shape.ambiguity_carrier_phase_samples == 0
        || shape.ephemeris_records == 0
        || shape.proposed_keyframes < 2
        || shape.proposed_keyframes > shape.imu_samples
        || shape.receiver_clock_nodes == 0
        || shape.receiver_clock_nodes > shape.proposed_keyframes
        || shape.troposphere_nodes == 0
        || shape.maximum_simultaneous_ambiguity_arcs == 0
        || u64::from(shape.maximum_simultaneous_ambiguity_arcs)
            > shape.ambiguity_carrier_phase_samples
        || shape.requested_output_epochs == 0
    {
        return Err(RawTightPreflightError::InvalidProblemShape);
    }

    let checked_add = |left: u64, right: u64| {
        left.checked_add(right)
            .ok_or(RawTightPreflightError::EstimateOverflow)
    };
    let checked_mul = |left: u64, right: u64| {
        left.checked_mul(right)
            .ok_or(RawTightPreflightError::EstimateOverflow)
    };

    let navigation_coordinates = checked_mul(shape.proposed_keyframes, 15)?;
    let receiver_clock_coordinates = checked_mul(shape.receiver_clock_nodes, 2)?;
    let troposphere_coordinates = shape.troposphere_nodes;
    let ambiguity_coordinates = u64::from(shape.maximum_simultaneous_ambiguity_arcs);
    let state_coordinate_count = checked_add(
        checked_add(navigation_coordinates, receiver_clock_coordinates)?,
        checked_add(
            checked_add(troposphere_coordinates, ambiguity_coordinates)?,
            u64::from(shape.inter_system_bias_coordinates),
        )?,
    )?;

    let mut factor_count = shape.proposed_keyframes - 1; // IMU preintegration
    for factors in [
        shape.pseudorange_samples,
        shape.ambiguity_carrier_phase_samples,
        shape.tdcp_samples,
        shape.doppler_samples,
        shape.receiver_clock_nodes.saturating_sub(1),
        shape.troposphere_nodes.saturating_sub(1),
        shape.base_or_correction_epochs,
        6, // gauge/state/bias/clock/tropo/inter-system priors
    ] {
        factor_count = checked_add(factor_count, factors)?;
    }

    // Dense differenced covariance blocks and ambiguity/keyframe coupling make
    // this deliberately more conservative than the solution-level graph.
    let measurement_nonzeros = checked_mul(factor_count, 144)?;
    let ambiguity_cross = checked_mul(
        ambiguity_coordinates,
        checked_mul(shape.proposed_keyframes.min(shape.rover_epochs), 18)?,
    )?;
    let estimated_sparse_nonzeros = checked_add(
        checked_add(measurement_nonzeros, ambiguity_cross)?,
        checked_mul(state_coordinate_count, 3)?,
    )?;

    let graph_bytes = checked_mul(factor_count, 640)?;
    let state_bytes = checked_mul(state_coordinate_count, 96)?;
    let sparse_bytes = checked_mul(estimated_sparse_nonzeros, 40)?;
    let integer_search_bytes = checked_mul(ambiguity_coordinates, ambiguity_coordinates)?
        .checked_mul(24)
        .ok_or(RawTightPreflightError::EstimateOverflow)?;
    let estimated_peak_memory_bytes = checked_add(
        checked_add(graph_bytes, state_bytes)?,
        checked_add(sparse_bytes, integer_search_bytes)?,
    )?;
    let estimated_temporary_storage_bytes = checked_add(
        checked_mul(shape.imu_samples, 112)?,
        checked_add(
            checked_mul(shape.raw_signal_samples, 256)?,
            checked_mul(estimated_sparse_nonzeros, 12)?,
        )?,
    )?;
    let estimated_output_bytes = checked_add(
        checked_mul(shape.requested_output_epochs, 640)?,
        checked_mul(shape.proposed_keyframes, 1_536)?,
    )?;

    let pass_limits = RawTightPassLimits {
        reverse_initialization_passes: 2,
        robust_reclassification_passes: 5,
        float_solver_iterations: 60,
        integer_validation_passes: 3,
        maximum_integer_candidates: 1_024,
    };
    let solver_sweeps = u64::from(pass_limits.reverse_initialization_passes)
        + u64::from(pass_limits.robust_reclassification_passes)
        + u64::from(pass_limits.float_solver_iterations)
        + u64::from(pass_limits.integer_validation_passes);
    let progress_work_units = checked_add(
        checked_mul(shape.imu_samples, 3)?,
        checked_add(
            checked_mul(shape.raw_signal_samples, 8)?,
            checked_mul(factor_count, solver_sweeps)?,
        )?,
    )?;

    Ok(RawTightResourceEstimate {
        estimation_model_revision: 1,
        proposed_keyframes: shape.proposed_keyframes,
        state_coordinate_count,
        factor_count,
        estimated_sparse_nonzeros,
        estimated_peak_memory_bytes,
        estimated_temporary_storage_bytes,
        estimated_output_bytes,
        progress_work_units,
        minimum_worker_count: 1,
        pass_limits,
    })
}

pub(super) fn enforce_limits(
    estimate: RawTightResourceEstimate,
    shape: RawTightProblemShape,
    registration: RawTightBackendRegistration,
    limits: OfflineResourceLimits,
) -> Result<(), RawTightPreflightError> {
    for (resource, required, available) in [
        (
            RawTightResourceKind::BackendSignalCapacity,
            shape.raw_signal_samples,
            registration.maximum_signal_samples,
        ),
        (
            RawTightResourceKind::BackendAmbiguityCapacity,
            u64::from(shape.maximum_simultaneous_ambiguity_arcs),
            u64::from(registration.maximum_simultaneous_ambiguity_arcs),
        ),
        (
            RawTightResourceKind::PeakMemory,
            estimate.estimated_peak_memory_bytes,
            limits.peak_memory_bytes,
        ),
        (
            RawTightResourceKind::TemporaryStorage,
            estimate.estimated_temporary_storage_bytes,
            limits.temporary_storage_bytes,
        ),
        (
            RawTightResourceKind::OutputStorage,
            estimate.estimated_output_bytes,
            limits.output_bytes,
        ),
        (
            RawTightResourceKind::WorkerCount,
            u64::from(estimate.minimum_worker_count),
            u64::from(limits.worker_count),
        ),
    ] {
        if required > available {
            return Err(RawTightPreflightError::InsufficientResource {
                resource,
                required,
                available,
            });
        }
    }
    if registration.maximum_integer_candidates < estimate.pass_limits.maximum_integer_candidates {
        return Err(RawTightPreflightError::InsufficientResource {
            resource: RawTightResourceKind::BackendIntegerCandidateCapacity,
            required: u64::from(estimate.pass_limits.maximum_integer_candidates),
            available: u64::from(registration.maximum_integer_candidates),
        });
    }
    if let Some(available) = limits.elapsed_work_limit {
        if estimate.progress_work_units > available {
            return Err(RawTightPreflightError::InsufficientResource {
                resource: RawTightResourceKind::ElapsedWork,
                required: estimate.progress_work_units,
                available,
            });
        }
    }
    Ok(())
}
