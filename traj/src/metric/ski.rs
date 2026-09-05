//! Offline hidden-Markov ski-state classification.

use super::{
    definition::{SkiHmmModel, SkiPlan, SkiState},
    geometry::{dot, norm},
    quality::metric_validity,
    report::{MetricError, SkiSegmentReport},
};
use crate::{quality::EstimateStage, time::SessionTime, trajectory::Trajectory};

#[cfg(feature = "offline")]
pub(super) fn ski_viterbi(
    trajectory: &Trajectory,
    plan: SkiPlan,
) -> Result<std::vec::Vec<SkiSegmentReport>, MetricError> {
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    let step =
        i64::try_from(plan.sample_period.as_ns()).map_err(|_| MetricError::InvalidDefinition)?;
    let minimum_segment_ns = i64::try_from(plan.minimum_segment_duration.as_ns())
        .map_err(|_| MetricError::InvalidDefinition)?;
    let mut samples = std::vec::Vec::new();
    let mut time_ns = span.start().as_ns();
    while time_ns <= span.end().as_ns() {
        let time = SessionTime::from_ns(time_ns);
        let state = trajectory.scalar_kinematics_at(time, plan.reference_point)?;
        let acceleration = state
            .acceleration_ecef_mps2
            .ok_or(MetricError::Unobservable)?;
        samples.push((
            time,
            [
                state.horizontal_speed_mps,
                state.vertical_speed_mps,
                norm(acceleration),
            ],
        ));
        time_ns = time_ns.saturating_add(step);
        if step == 0 {
            return Err(MetricError::InvalidDefinition);
        }
    }
    if samples.is_empty() {
        return Ok(std::vec::Vec::new());
    }

    let mut score = std::vec![[f64::NEG_INFINITY; 5]; samples.len()];
    let mut parent = std::vec![[0u8; 5]; samples.len()];
    for state in SkiState::ALL {
        score[0][state.index()] = plan.model.initial_log_probability[state.index()]
            + emission_score(plan.model, state, samples[0].1);
    }
    for index in 1..samples.len() {
        for state in SkiState::ALL {
            let mut best = f64::NEG_INFINITY;
            let mut best_parent = 0usize;
            for previous in SkiState::ALL {
                let candidate = score[index - 1][previous.index()]
                    + plan.model.transition_log_probability[previous.index()][state.index()];
                if candidate > best {
                    best = candidate;
                    best_parent = previous.index();
                }
            }
            score[index][state.index()] =
                best + emission_score(plan.model, state, samples[index].1);
            parent[index][state.index()] = best_parent as u8;
        }
    }

    let mut last_state = 0usize;
    for state in 1..5 {
        if score[samples.len() - 1][state] > score[samples.len() - 1][last_state] {
            last_state = state;
        }
    }
    let mut path = std::vec![0u8; samples.len()];
    path[samples.len() - 1] = last_state as u8;
    for index in (1..samples.len()).rev() {
        path[index - 1] = parent[index][path[index] as usize];
    }

    let mut reports = std::vec::Vec::new();
    let mut begin = 0usize;
    for index in 1..=path.len() {
        if index == path.len() || path[index] != path[begin] {
            let start = samples[begin].0;
            let end = samples[index.saturating_sub(1)].0;
            if end.as_ns().saturating_sub(start.as_ns()) >= minimum_segment_ns {
                let state = SkiState::ALL[path[begin] as usize];
                let state_score = score[index.saturating_sub(1)][state.index()];
                let mut second = f64::NEG_INFINITY;
                for alternative in SkiState::ALL {
                    if alternative != state {
                        second = second.max(score[index.saturating_sub(1)][alternative.index()]);
                    }
                }
                reports.push(SkiSegmentReport {
                    definition: plan.definition,
                    state,
                    start,
                    end,
                    confidence: logistic(state_score - second),
                    reference_point: plan.reference_point,
                    stage: EstimateStage::Finalized,
                    validity: metric_validity(
                        trajectory.conservative_quality_over_span(start, end)?,
                    ),
                });
            }
            begin = index;
        }
    }
    Ok(reports)
}

#[cfg(feature = "offline")]
fn emission_score(model: SkiHmmModel, state: SkiState, feature: [f64; 3]) -> f64 {
    model.emission_bias[state.index()] + dot(model.emission_weight[state.index()], feature)
}

#[cfg(feature = "offline")]
fn logistic(value: f64) -> f64 {
    if value >= 0.0 {
        1.0 / (1.0 + (-value).exp())
    } else {
        let exponential = value.exp();
        exponential / (1.0 + exponential)
    }
}
