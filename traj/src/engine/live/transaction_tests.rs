//! Regression tests for live step transaction tests.

use super::*;
use crate::ids::SourceId;

#[test]
fn rejected_ingest_can_restore_updated_and_new_source_sequences() {
    let mut sequences = heapless::Vec::<SourceSequence, MAX_LIVE_SOURCES>::new();
    sequences
        .push(SourceSequence {
            source: SourceId::new(1),
            latest: 5,
        })
        .unwrap();

    let updated =
        begin_source_sequence(&mut sequences, ObservationId::new(SourceId::new(1), 8)).unwrap();
    assert_eq!(sequences[0].latest, 8);
    rollback_source_sequence(&mut sequences, updated);
    assert_eq!(
        sequences.as_slice(),
        &[SourceSequence {
            source: SourceId::new(1),
            latest: 5,
        }]
    );

    let added =
        begin_source_sequence(&mut sequences, ObservationId::new(SourceId::new(2), 1)).unwrap();
    assert_eq!(sequences.len(), 2);
    rollback_source_sequence(&mut sequences, added);
    assert_eq!(sequences.len(), 1);

    assert!(matches!(
        begin_source_sequence(&mut sequences, ObservationId::new(SourceId::new(1), 5),),
        Err(StepError::DuplicateObservation {
            source,
            sequence: 5,
        }) if source == SourceId::new(1)
    ));
    assert_eq!(sequences[0].latest, 5);
}
