//! The exact writable layout required by the fixed-address two-slot updater.
use super::{
    state::MODE_DATA,
    update::{APP_SIZE, APP_START, Error, OTA_DATA, SECTOR},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Partition {
    pub kind: u8,
    pub subtype: u8,
    pub offset: u32,
    pub size: u32,
    pub flags: u32,
}

pub(crate) const EXPECTED: [Partition; 4] = [
    Partition {
        kind: 1,
        subtype: 0,
        offset: OTA_DATA,
        size: 2 * SECTOR,
        flags: 0,
    },
    Partition {
        kind: 1,
        subtype: 6,
        offset: MODE_DATA,
        size: SECTOR,
        flags: 0,
    },
    Partition {
        kind: 0,
        subtype: 0x10,
        offset: APP_START,
        size: APP_SIZE,
        flags: 0,
    },
    Partition {
        kind: 0,
        subtype: 0x11,
        offset: APP_START + APP_SIZE,
        size: APP_SIZE,
        flags: 0,
    },
];

/// Order and labels do not affect addresses; every required entry must occur
/// exactly once, with no encrypted, read-only, unknown or overlapping regions.
pub(crate) fn validate(entries: impl IntoIterator<Item = Partition>) -> Result<(), Error> {
    let mut seen = 0_u8;
    for entry in entries {
        let index = EXPECTED
            .iter()
            .position(|expected| *expected == entry)
            .ok_or(Error::Layout)?;
        let bit = 1 << index;
        if seen & bit != 0 {
            return Err(Error::Layout);
        }
        seen |= bit;
    }
    if seen == 0b1111 {
        Ok(())
    } else {
        Err(Error::Layout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_only_the_complete_fixed_layout_independent_of_order() {
        assert_eq!(validate(EXPECTED), Ok(()));
        let mut reversed = EXPECTED;
        reversed.reverse();
        assert_eq!(validate(reversed), Ok(()));
        assert_eq!(validate(EXPECTED[..3].iter().copied()), Err(Error::Layout));
        let mut duplicate = EXPECTED;
        duplicate[3] = duplicate[2];
        assert_eq!(validate(duplicate), Err(Error::Layout));
    }

    #[test]
    fn four_entries_with_changed_geometry_type_or_flags_are_rejected() {
        for index in 0..4 {
            for field in 0..5 {
                let mut entries = EXPECTED;
                match field {
                    0 => entries[index].kind ^= 1,
                    1 => entries[index].subtype ^= 1,
                    2 => entries[index].offset += SECTOR,
                    3 => entries[index].size += SECTOR,
                    _ => entries[index].flags = 1 << index,
                }
                assert_eq!(
                    validate(entries),
                    Err(Error::Layout),
                    "entry {index}, field {field}"
                );
            }
        }
    }
}
