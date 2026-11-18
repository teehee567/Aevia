use super::*;
use std::{collections::BTreeMap, vec::Vec};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Injected;

#[derive(Clone)]
struct Disk {
    sectors: u64,
    data: BTreeMap<u64, [u8; 512]>,
    reads: usize,
    bulk_reads: Vec<(u64, usize)>,
    fail_bulk_read: bool,
    events: usize,
    fail_at: Option<usize>,
    after_write: bool,
    torn_bytes: Option<usize>,
}
impl Disk {
    fn get(&self, sector: u64) -> [u8; 512] {
        self.data.get(&sector).copied().unwrap_or([0; 512])
    }
    fn put(&mut self, sector: u64, data: [u8; 512]) {
        assert!(sector < self.sectors);
        self.data.insert(sector, data);
    }
}
impl BlockStorage for Disk {
    type Error = Injected;
    fn sector_count(&self) -> u64 {
        self.sectors
    }
    fn read_sector(&mut self, sector: u64, data: &mut [u8; 512]) -> Result<(), Injected> {
        assert!(sector < self.sectors);
        self.reads += 1;
        *data = self.get(sector);
        Ok(())
    }
    fn write_sector(&mut self, sector: u64, data: &[u8; 512]) -> Result<(), Injected> {
        let fail = self.fail_at == Some(self.events);
        self.events += 1;
        if fail {
            if let Some(bytes) = self.torn_bytes {
                let mut previous = self.get(sector);
                previous[..bytes].copy_from_slice(&data[..bytes]);
                self.put(sector, previous);
            } else if self.after_write {
                self.put(sector, *data);
            }
            return Err(Injected);
        }
        self.put(sector, *data);
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Injected> {
        let fail = self.fail_at == Some(self.events);
        self.events += 1;
        if fail { Err(Injected) } else { Ok(()) }
    }
}

impl ReadSectors for Disk {
    fn read_sectors(&mut self, first: u64, data: &mut [u8]) -> Result<(), Injected> {
        self.bulk_reads.push((first, data.len()));
        if self.fail_bulk_read {
            return Err(Injected);
        }
        for (index, bytes) in data.chunks_exact_mut(512).enumerate() {
            self.read_sector(first + index as u64, bytes.try_into().unwrap())?;
        }
        Ok(())
    }
}

/// A sparse, specification-shaped image. Its ordinary USER.BIN and root entry
/// are unrelated to recording and must survive every operation/failure.
fn fixture(large: bool, partitioned: bool) -> (Disk, Geometry) {
    let start = if partitioned { 2048 } else { 0 };
    let shift = if large { 8 } else { 3 };
    let length = if large { 249_997_824 } else { 32_768 };
    let heap = if large { 8192 } else { 64 };
    let clusters = ((length - heap) >> shift) as u32;
    let fat_len = (u64::from(clusters) + 2).div_ceil(128) as u32;
    let geometry = Geometry {
        start,
        length,
        fat: start + 24,
        heap: start + heap,
        clusters,
        sectors_per_cluster: 1 << shift,
        root: 4,
        serial: 0x12345678,
    };
    let mut disk = Disk {
        sectors: start + length,
        data: BTreeMap::new(),
        reads: 0,
        bulk_reads: Vec::new(),
        fail_bulk_read: false,
        events: 0,
        fail_at: None,
        after_write: false,
        torn_bytes: None,
    };
    let mut boot = [0; 512];
    boot[..3].copy_from_slice(&[0xeb, 0x76, 0x90]);
    boot[3..11].copy_from_slice(b"EXFAT   ");
    boot[64..72].copy_from_slice(&start.to_le_bytes());
    boot[72..80].copy_from_slice(&length.to_le_bytes());
    boot[80..84].copy_from_slice(&24_u32.to_le_bytes());
    boot[84..88].copy_from_slice(&fat_len.to_le_bytes());
    boot[88..92].copy_from_slice(&(heap as u32).to_le_bytes());
    boot[92..96].copy_from_slice(&clusters.to_le_bytes());
    boot[96..100].copy_from_slice(&4_u32.to_le_bytes());
    boot[100..104].copy_from_slice(&geometry.serial.to_le_bytes());
    boot[104..106].copy_from_slice(&0x0100_u16.to_le_bytes());
    boot[108] = 9;
    boot[109] = shift;
    boot[110] = 1;
    boot[112] = 255;
    boot[510..512].copy_from_slice(&[0x55, 0xaa]);
    for offset in [0, 12] {
        disk.put(start + offset, boot);
        for sector in 1..=8 {
            let mut extended = [0; 512];
            extended[508..512].copy_from_slice(&0xaa550000_u32.to_le_bytes());
            disk.put(start + offset + sector, extended);
        }
        refresh_boot_checksum(&mut disk, start + offset);
    }
    if partitioned {
        let mut mbr = [0; 512];
        mbr[450] = 0x07;
        mbr[454..458].copy_from_slice(&(start as u32).to_le_bytes());
        mbr[458..462].copy_from_slice(&(length as u32).to_le_bytes());
        mbr[510..512].copy_from_slice(&[0x55, 0xaa]);
        disk.put(0, mbr);
    }
    let mut fat = [0; 512];
    for cluster in 0..=5 {
        fat[cluster * 4..cluster * 4 + 4].copy_from_slice(&0xffffffff_u32.to_le_bytes());
    }
    disk.put(geometry.fat, fat);
    let mut bitmap = [0; 512];
    bitmap[0] = 15; // bitmap, upcase, root, USER.BIN
    disk.put(geometry.cluster_sector(2), bitmap);
    let mut upper = [0; 512];
    for codepoint in 0_u16..128 {
        upper[usize::from(codepoint) * 2..usize::from(codepoint) * 2 + 2]
            .copy_from_slice(&ascii_upper(codepoint).to_le_bytes());
    }
    upper[256..258].copy_from_slice(&0xffff_u16.to_le_bytes());
    upper[258..260].copy_from_slice(&65407_u16.to_le_bytes());
    upper[260..262].copy_from_slice(&0xffff_u16.to_le_bytes());
    let upcase_checksum = upper[..262].iter().fold(0_u32, |sum, &v| {
        sum.rotate_right(1).wrapping_add(u32::from(v))
    });
    disk.put(geometry.cluster_sector(3), upper);
    let mut root = [0; 512];
    root[0] = 0x81;
    root[20..24].copy_from_slice(&2_u32.to_le_bytes());
    root[24..32].copy_from_slice(&u64::from(clusters).div_ceil(8).to_le_bytes());
    root[32] = 0x82;
    root[36..40].copy_from_slice(&upcase_checksum.to_le_bytes());
    root[52..56].copy_from_slice(&3_u32.to_le_bytes());
    root[56..64].copy_from_slice(&262_u64.to_le_bytes());
    root[64] = 0x83; // empty volume label
    let mut unrelated = directory_set(0x1234, 5, 512);
    unrelated[35] = 8;
    unrelated[66..96].fill(0);
    let name: Vec<u16> = "USER.BIN".encode_utf16().collect();
    for (i, v) in name.iter().enumerate() {
        unrelated[66 + i * 2..68 + i * 2].copy_from_slice(&v.to_le_bytes());
    }
    unrelated[36..38].copy_from_slice(&name_hash(&name).to_le_bytes());
    let checksum = set_checksum(0, &unrelated, true);
    unrelated[2..4].copy_from_slice(&checksum.to_le_bytes());
    root[96..192].copy_from_slice(&unrelated);
    disk.put(geometry.cluster_sector(4), root);
    disk.put(geometry.cluster_sector(5), [0x5a; 512]);
    (disk, geometry)
}

fn refresh_boot_checksum(disk: &mut Disk, start: u64) {
    let mut checksum = 0_u32;
    for sector in 0..11 {
        for (index, byte) in disk.get(start + sector).into_iter().enumerate() {
            if sector == 0 && matches!(index, 106 | 107 | 112) {
                continue;
            }
            checksum = checksum.rotate_right(1).wrapping_add(u32::from(byte));
        }
    }
    let mut data = [0; 512];
    for word in data.as_chunks_mut::<4>().0 {
        word.copy_from_slice(&checksum.to_le_bytes());
    }
    disk.put(start + 11, data);
}

#[test]
fn mounts_existing_superfloppy_and_partitioned_128gb_without_writes() {
    for (large, partitioned) in [(false, false), (false, true), (true, true)] {
        let (mut disk, geometry) = fixture(large, partitioned);
        let original = disk.data.clone();
        let mut callbacks = 0;
        let fs = FileSystem::mount(&mut disk, &mut || callbacks += 1).unwrap();
        assert_eq!(fs.geometry, geometry);
        assert!(fs.writable());
        assert_eq!(fs.kind(), FileSystemKind::ExFat);
        assert_eq!(disk.data, original);
        assert_eq!(disk.events, 0);
        assert_eq!(callbacks, disk.reads);
    }
}

#[test]
fn reserves_real_two_gib_extent_and_streams_without_metadata_changes() {
    let (mut disk, geometry) = fixture(true, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let original_user = disk.get(geometry.cluster_sector(5));
    let original_user_entry = disk.get(geometry.cluster_sector(4))[96..192].to_vec();
    let file = fs.create(&mut disk, 1, 4_194_304, &mut || {}).unwrap();
    assert_eq!(file.capacity_bytes(), 2 * 1024 * 1024 * 1024);
    assert_eq!(file.first_sector(), geometry.cluster_sector(6));
    assert_eq!(disk.get(file.first_sector()), [0; 512]);
    assert_eq!(
        disk.get(geometry.cluster_sector(4))[96..192],
        original_user_entry
    );
    assert_eq!(disk.get(geometry.cluster_sector(5)), original_user);
    let before = disk.data.clone();
    fs.write_sector(&mut disk, &file, 0, &[0xa5; 512]).unwrap();
    fs.write_sector(&mut disk, &file, 1, &[0x33; 512]).unwrap();
    fs.flush(&mut disk).unwrap();
    for (sector, data) in before {
        if sector != file.first_sector() {
            assert_eq!(disk.get(sector), data);
        }
    }
    let reads = disk.reads;
    assert_eq!(fs.find(&mut disk, 1, &mut || {}).unwrap(), Some(file));
    assert!(
        disk.reads - reads < 30,
        "2 GiB inspection must read bitmap sectors, not every cluster"
    );
    let mut data = [0; 512];
    fs.read_sector(&mut disk, &file, 1, &mut data).unwrap();
    assert_eq!(data, [0x33; 512]);
    assert_eq!(
        fs.read_sector(&mut disk, &file, file.sector_count(), &mut data),
        Err(Error::Bounds)
    );
    assert_eq!(
        fs.write_sector(&mut disk, &file, file.sector_count(), &[0; 512]),
        Err(Error::Bounds)
    );
    assert_eq!(disk.get(geometry.cluster_sector(5)), original_user);
}

#[test]
fn multiple_recordings_cross_directory_sector_end_without_losing_earlier_files() {
    let (mut disk, geometry) = fixture(false, true);
    let original = disk.get(geometry.cluster_sector(4))[..192].to_vec();
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    for id in 1..=24 {
        let file = fs.create(&mut disk, id, 8, &mut || {}).unwrap();
        fs.write_sector(&mut disk, &file, 0, &[id as u8; 512])
            .unwrap();
    }
    let fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let mut ids = Vec::new();
    fs.visit(&mut disk, &mut || {}, &mut |f| ids.push(f.id()))
        .unwrap();
    assert_eq!(ids, (1..=24).collect::<Vec<_>>());
    assert_eq!(disk.get(geometry.cluster_sector(4))[..192], original);
    for id in ids {
        let file = fs.find(&mut disk, id, &mut || {}).unwrap().unwrap();
        assert_eq!(disk.get(file.first_sector()), [id as u8; 512]);
    }
}

#[test]
fn every_allocation_failure_preserves_unrelated_file_and_never_publishes_free_clusters() {
    let (base, geometry) = fixture(false, true);
    let mut success = base.clone();
    let mut fs = FileSystem::mount(&mut success, &mut || {}).unwrap();
    fs.create(&mut success, 1, 64, &mut || {}).unwrap();
    for after_write in [false, true] {
        for fail_at in 0..success.events {
            let mut disk = base.clone();
            let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
            disk.fail_at = Some(fail_at);
            disk.after_write = after_write;
            assert!(fs.create(&mut disk, 1, 64, &mut || {}).is_err());
            assert!(!fs.writable());
            assert_eq!(
                disk.get(geometry.cluster_sector(5)),
                base.get(geometry.cluster_sector(5))
            );
            assert_eq!(
                disk.get(geometry.cluster_sector(4))[..192],
                base.get(geometry.cluster_sector(4))[..192]
            );
            assert_eq!(disk.get(0), base.get(0));
            disk.fail_at = None;
            let remounted = FileSystem::mount(&mut disk, &mut || {}).unwrap();
            if let Some(file) = remounted.find(&mut disk, 1, &mut || {}).unwrap() {
                assert_eq!(disk.get(file.first_sector()), [0; 512]);
                // find() validates every referenced allocation bit.
                assert_eq!(file.sector_count(), 64);
            }
        }
    }
}

#[test]
fn torn_directory_sector_is_rejected_or_recoverable_read_only_without_mutation() {
    let (mut base, geometry) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut base, &mut || {}).unwrap();
    let previous_file = fs.create(&mut base, 7, 64, &mut || {}).unwrap();
    fs.write_sector(&mut base, &previous_file, 0, &[7; 512])
        .unwrap();
    base.events = 0;
    // events: dirty write/flush, bitmap write/flush, header write/flush, directory write.
    for bytes in [1, 288, 289, 290, 291, 320, 352, 383, 384, 416, 511, 512] {
        let mut disk = base.clone();
        let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
        disk.fail_at = Some(6);
        disk.torn_bytes = Some(bytes);
        assert!(fs.create(&mut disk, 1, 64, &mut || {}).is_err());
        assert_eq!(
            disk.get(geometry.cluster_sector(4))[..192],
            base.get(geometry.cluster_sector(4))[..192]
        );
        assert_eq!(disk.get(geometry.cluster_sector(5)), [0x5a; 512]);
        disk.fail_at = None;
        let before = disk.data.clone();
        let fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
        assert!(!fs.writable());
        let recovered = fs.find(&mut disk, 7, &mut || {}).unwrap().unwrap();
        assert_eq!(recovered, previous_file);
        let mut data = [0; 512];
        fs.read_sector(&mut disk, &recovered, 0, &mut data).unwrap();
        assert_eq!(data, [7; 512]);
        assert_eq!(disk.data, before);
    }
}

#[test]
fn writes_require_the_current_new_file_and_exact_next_sector() {
    let (mut disk, _) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let first = fs.create(&mut disk, 1, 16, &mut || {}).unwrap();
    assert_eq!(
        fs.write_sector(&mut disk, &first, 1, &[1; 512]),
        Err(Error::Bounds)
    );
    fs.write_sector(&mut disk, &first, 0, &[1; 512]).unwrap();
    assert_eq!(
        fs.write_sector(&mut disk, &first, 0, &[2; 512]),
        Err(Error::Bounds)
    );
    let second = fs.create(&mut disk, 2, 16, &mut || {}).unwrap();
    assert_eq!(
        fs.write_sector(&mut disk, &first, 1, &[3; 512]),
        Err(Error::Bounds)
    );
    fs.write_sector(&mut disk, &second, 0, &[2; 512]).unwrap();
    let mut mounted = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let found = mounted.find(&mut disk, 2, &mut || {}).unwrap().unwrap();
    assert_eq!(
        mounted.write_sector(&mut disk, &found, 1, &[0; 512]),
        Err(Error::Bounds)
    );
    assert_eq!(disk.get(first.first_sector()), [1; 512]);
    assert_eq!(disk.get(second.first_sector()), [2; 512]);
}

#[test]
fn corrupted_geometry_boot_checksum_chain_and_metadata_allocation_fail_without_writes() {
    let (base, geometry) = fixture(false, true);
    for scenario in 0..6 {
        let mut disk = base.clone();
        match scenario {
            0 => {
                let mut boot = disk.get(geometry.start);
                boot[100] ^= 1;
                disk.put(geometry.start, boot);
            }
            1 => {
                let mut backup = disk.get(geometry.start + 23);
                backup[99] ^= 1;
                disk.put(geometry.start + 23, backup);
            }
            2 => {
                let mut fat = disk.get(geometry.fat);
                fat[16..20].copy_from_slice(&4_u32.to_le_bytes());
                disk.put(geometry.fat, fat);
            }
            3 => {
                let mut bitmap = disk.get(geometry.heap);
                bitmap[0] &= !(1 << 2);
                disk.put(geometry.heap, bitmap);
            }
            4 => {
                let mut boot = disk.get(geometry.start);
                boot[92..96].copy_from_slice(&u32::MAX.to_le_bytes());
                disk.put(geometry.start, boot);
                refresh_boot_checksum(&mut disk, geometry.start);
            }
            5 => {
                let mut table = disk.get(geometry.cluster_sector(3));
                table[0] ^= 1;
                disk.put(geometry.cluster_sector(3), table);
            }
            _ => unreachable!(),
        }
        let original = disk.data.clone();
        assert!(
            FileSystem::mount(&mut disk, &mut || {}).is_err(),
            "scenario {scenario}"
        );
        assert!(disk.reads < 200);
        assert_eq!(disk.events, 0);
        assert_eq!(disk.data, original);
    }
}

#[test]
fn dirty_volume_allows_retrieval_but_never_clears_dirty_or_creates() {
    let (mut disk, geometry) = fixture(false, false);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let file = fs.create(&mut disk, 9, 16, &mut || {}).unwrap();
    fs.write_sector(&mut disk, &file, 0, &[9; 512]).unwrap();
    let mut boot = disk.get(geometry.start);
    boot[106] |= 2;
    disk.put(geometry.start, boot);
    let original = disk.data.clone();
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    assert!(!fs.writable());
    assert_eq!(fs.find(&mut disk, 9, &mut || {}).unwrap(), Some(file));
    assert_eq!(
        fs.create(&mut disk, 10, 8, &mut || {}),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        fs.write_sector(&mut disk, &file, 0, &[0; 512]),
        Err(Error::ReadOnly)
    );
    let mut sector = [0; 512];
    fs.read_sector(&mut disk, &file, 0, &mut sector).unwrap();
    assert_eq!(sector, [9; 512]);
    assert_eq!(disk.data, original);
}

#[test]
fn name_collision_case_folding_and_invalid_requests_do_not_mutate_existing_data() {
    let (mut disk, geometry) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let file = fs.create(&mut disk, 0xabcdef12, 16, &mut || {}).unwrap();
    let mut root = disk.get(geometry.cluster_sector(4));
    for index in 0..14 {
        let offset = 192 + 66 + index * 2;
        if root[offset].is_ascii_uppercase() {
            root[offset] += 32;
        }
    }
    let checksum = set_checksum(0, &root[192..288], true);
    root[194..196].copy_from_slice(&checksum.to_le_bytes());
    disk.put(geometry.cluster_sector(4), root);
    let before = disk.data.clone();
    assert_eq!(
        fs.find(&mut disk, file.id(), &mut || {}).unwrap(),
        Some(file)
    );
    assert_eq!(
        fs.create(&mut disk, file.id(), 16, &mut || {}),
        Err(Error::AlreadyExists)
    );
    assert_eq!(
        fs.create(&mut disk, 0, 16, &mut || {}),
        Err(Error::InvalidId)
    );
    assert_eq!(fs.create(&mut disk, 1, 0, &mut || {}), Err(Error::Bounds));
    assert_eq!(disk.data, before);
}

#[test]
fn full_card_and_full_root_fail_before_marking_volume_dirty() {
    let (mut disk, geometry) = fixture(false, true);
    let mut bitmap = disk.get(geometry.heap);
    bitmap.fill(0xff);
    disk.put(geometry.heap, bitmap);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let original = disk.data.clone();
    assert_eq!(fs.create(&mut disk, 1, 8, &mut || {}), Err(Error::NoSpace));
    assert_eq!(disk.data, original);
    assert_eq!(disk.events, 0);
    let (mut disk, geometry) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    // Benign primary entries consume the existing root allocation.
    for index in 0..geometry.sectors_per_cluster {
        let address = geometry.cluster_sector(4) + u64::from(index);
        let mut data = disk.get(address);
        for entry in data[if index == 0 { 192 } else { 0 }..]
            .as_chunks_mut::<32>()
            .0
        {
            entry[0] = 0xa0;
        }
        disk.put(address, data);
    }
    let original = disk.data.clone();
    assert_eq!(
        fs.create(&mut disk, 1, 8, &mut || {}),
        Err(Error::DirectoryFull)
    );
    assert_eq!(disk.data, original);
    assert_eq!(disk.events, 0);
}

#[test]
fn bounded_reads_delegate_one_contiguous_extent_without_mutating_existing_files() {
    let (mut disk, _) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let file = fs.create(&mut disk, 1, 16, &mut || {}).unwrap();
    for index in 0..16 {
        fs.write_sector(&mut disk, &file, index, &[0x40 + index as u8; 512])
            .unwrap();
    }
    let original = disk.data.clone();
    let writes = disk.events;
    let mut data = [0; 4096];
    fs.read_sectors(&mut disk, &file, 3, &mut data).unwrap();
    assert_eq!(disk.bulk_reads, [(file.first_sector() + 3, 4096)]);
    for (index, sector) in data.chunks_exact(512).enumerate() {
        assert_eq!(sector, [0x43 + index as u8; 512]);
    }
    // A remounted read-only volume still permits complete prior-file retrieval.
    fs.writable = false;
    fs.read_sectors(&mut disk, &file, 8, &mut data).unwrap();
    assert_eq!(&data[7 * 512..], [0x4f; 512]);
    disk.fail_bulk_read = true;
    assert_eq!(
        fs.read_sectors(&mut disk, &file, 0, &mut data),
        Err(Error::Storage(Injected))
    );
    assert_eq!(disk.data, original);
    assert_eq!(disk.events, writes);
}

#[test]
fn bounded_reads_reject_entire_bad_range_or_foreign_file_before_any_io() {
    let (mut disk, _) = fixture(false, true);
    let mut fs = FileSystem::mount(&mut disk, &mut || {}).unwrap();
    let file = fs.create(&mut disk, 1, 16, &mut || {}).unwrap();
    let reads = disk.reads;
    let writes = disk.events;
    let mut data = [0xa5; 4608];
    for length in [0, 1, 511, 513, 4095, 4097, 4608] {
        assert_eq!(
            fs.read_sectors(&mut disk, &file, 0, &mut data[..length]),
            Err(Error::Bounds)
        );
    }
    for first in [9, 16, u32::MAX - 3, u32::MAX] {
        assert_eq!(
            fs.read_sectors(&mut disk, &file, first, &mut data[..4096]),
            Err(Error::Bounds)
        );
    }
    let mut foreign = file;
    foreign.volume_serial ^= 1;
    assert_eq!(
        fs.read_sectors(&mut disk, &foreign, 0, &mut data[..4096]),
        Err(Error::Bounds)
    );
    assert!(disk.bulk_reads.is_empty());
    assert_eq!(disk.reads, reads);
    assert_eq!(disk.events, writes);
    assert_eq!(data, [0xa5; 4608]);
}
