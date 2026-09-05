use super::*;

#[test]
fn shared_boundary_is_owned_by_right_segment() {
    let mut trajectory = trajectory();
    let previous = *trajectory.segments.last().unwrap();
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(2_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 20.0, 0.0).unwrap(),
        ..previous.end
    };
    trajectory.push_hermite_segment(previous.end, end).unwrap();
    let (index, parameter) = trajectory
        .locate(SessionTime::from_ns(1_000_000_000))
        .unwrap();
    assert_eq!(index, 1);
    assert_eq!(parameter, 0.0);
}

#[cfg(not(feature = "offline"))]
#[test]
fn embedded_storage_accepts_full_live_history_and_rejects_one_more() {
    let mut trajectory = trajectory();
    for segment_number in 1..MAX_EMBEDDED_TRAJECTORY_SEGMENTS {
        let previous = *trajectory.segments.last().unwrap();
        let end = TrajectoryKnot {
            time: SessionTime::from_ns((segment_number as i64 + 1) * 1_000_000_000),
            position_ecef: EcefPosition::new(
                6_378_137.0,
                (segment_number as f64 + 1.0) * 10.0,
                0.0,
            )
            .unwrap(),
            ..previous.end
        };
        trajectory.push_hermite_segment(previous.end, end).unwrap();
    }
    assert_eq!(trajectory.segment_count(), MAX_EMBEDDED_TRAJECTORY_SEGMENTS);

    let previous = *trajectory.segments.last().unwrap();
    let overflow_end = TrajectoryKnot {
        time: SessionTime::from_ns((MAX_EMBEDDED_TRAJECTORY_SEGMENTS as i64 + 1) * 1_000_000_000),
        position_ecef: EcefPosition::new(
            6_378_137.0,
            (MAX_EMBEDDED_TRAJECTORY_SEGMENTS as f64 + 1.0) * 10.0,
            0.0,
        )
        .unwrap(),
        ..previous.end
    };
    assert_eq!(
        trajectory.push_hermite_segment(previous.end, overflow_end),
        Err(ValidationError::CapacityExceeded)
    );
    assert_eq!(trajectory.segment_count(), MAX_EMBEDDED_TRAJECTORY_SEGMENTS);
}

#[test]
fn rejected_rolling_segment_does_not_evict_committed_history() {
    let mut trajectory = trajectory();
    for segment_number in 1..MAX_EMBEDDED_TRAJECTORY_SEGMENTS {
        let previous = *trajectory.segments.last().unwrap();
        let end = TrajectoryKnot {
            time: SessionTime::from_ns((segment_number as i64 + 1) * 1_000_000_000),
            position_ecef: EcefPosition::new(
                6_378_137.0,
                (segment_number as f64 + 1.0) * 10.0,
                0.0,
            )
            .unwrap(),
            ..previous.end
        };
        trajectory.push_hermite_segment(previous.end, end).unwrap();
    }
    let before = trajectory.span().unwrap();
    let previous = *trajectory.segments.last().unwrap();
    let invalid_start = previous.start;
    let invalid_end = TrajectoryKnot {
        time: SessionTime::from_ns(previous.end.time.as_ns() + 1_000_000_000),
        ..previous.end
    };

    assert_eq!(
        trajectory.push_rolling_hermite_segment(invalid_start, invalid_end),
        Err(ValidationError::InvalidTimeSpan)
    );
    assert_eq!(trajectory.segment_count(), MAX_EMBEDDED_TRAJECTORY_SEGMENTS);
    assert_eq!(trajectory.span(), Some(before));
}

#[test]
fn query_outside_span_reports_actual_bounds() {
    let trajectory = trajectory();
    assert!(matches!(
        trajectory.state_at(SessionTime::from_ns(-1), ReferencePointId::new(1)),
        Err(QueryError::OutsideAvailableSpan {
            earliest: Some(_),
            latest: Some(_),
            ..
        })
    ));
}
