//! Restricted recording-file storage on an existing exFAT volume.
//!
//! Geometry, checksums, directory sets and allocation follow Microsoft's exFAT
//! specification: <https://learn.microsoft.com/windows/win32/fileio/exfat-specification>.
//! This is not a general filesystem: it never formats, deletes, truncates, or
//! changes an unrelated file. New recordings reserve one contiguous extent and
//! publish their complete capacity before streaming. The recording frame format
//! determines how much of that capacity contains committed data.
//!
//! Allocation is ordered: dirty flag -> bitmap -> empty recording header ->
//! directory -> clean flag, with storage barriers between stages. An interrupted
//! metadata operation can leak reserved clusters, and leaves the volume dirty
//! and read-only on remount. Ordinary streaming does not change FS metadata.
//! CRC framing detects torn data; consumer SD cards do not promise power-loss
//! atomic sectors or durable writes beyond their own controller guarantees.

use aevia_device_hal::interfaces::{BlockStorage, ReadSectors};

const SECTOR: u64 = 512;
const MAX_ROOT_SECTORS: u32 = 8192;
const MAX_BITMAP_SECTORS: u32 = 65_536;
const MAX_CHAIN_CLUSTERS: u32 = 65_536;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FileSystemKind {
    ExFat,
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Storage(E),
    Unsupported,
    Corrupt,
    Bounds,
    ScanLimit,
    ReadOnly,
    NoSpace,
    DirectoryFull,
    AlreadyExists,
    InvalidId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RecordingFile {
    id: u32,
    first_sector: u64,
    sector_count: u32,
    volume_start: u64,
    volume_serial: u32,
}

impl RecordingFile {
    pub const fn id(self) -> u32 {
        self.id
    }
    pub const fn first_sector(self) -> u64 {
        self.first_sector
    }
    pub const fn sector_count(self) -> u32 {
        self.sector_count
    }
    pub const fn capacity_bytes(self) -> u64 {
        self.sector_count as u64 * SECTOR
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Geometry {
    start: u64,
    length: u64,
    fat: u64,
    heap: u64,
    clusters: u32,
    sectors_per_cluster: u32,
    root: u32,
    serial: u32,
}

#[derive(Clone, Copy, Debug)]
struct Stream {
    first: u32,
    bytes: u64,
}

#[derive(Debug)]
pub struct FileSystem {
    geometry: Geometry,
    bitmap: Stream,
    writable: bool,
    active: Option<(RecordingFile, u32)>,
}

impl FileSystem {
    /// Mount without writing. `progress` runs after every metadata sector I/O;
    /// firmware can feed supervision/yield without holding a shared mutex.
    pub fn mount<B: BlockStorage>(
        storage: &mut B,
        progress: &mut impl FnMut(),
    ) -> Result<Self, Error<B::Error>> {
        let mut boot = [0; 512];
        read(storage, 0, &mut boot, progress)?;
        let (start, limit) = if &boot[3..11] == b"EXFAT   " {
            (0, storage.sector_count())
        } else {
            if u16le(&boot, 510) != 0xaa55 {
                return Err(Error::Unsupported);
            }
            let mut selected = None;
            for n in 0..4 {
                let offset = 446 + n * 16;
                if boot[offset + 4] == 0xee {
                    return Err(Error::Unsupported);
                }
                if boot[offset + 4] != 0x07 {
                    continue;
                }
                let start = u64::from(u32le(&boot, offset + 8));
                let length = u64::from(u32le(&boot, offset + 12));
                if start == 0
                    || length == 0
                    || start
                        .checked_add(length)
                        .is_none_or(|end| end > storage.sector_count())
                {
                    return Err(Error::Corrupt);
                }
                let mut candidate = [0; 512];
                read(storage, start, &mut candidate, progress)?;
                if &candidate[3..11] == b"EXFAT   " {
                    if selected.is_some() {
                        return Err(Error::Unsupported);
                    }
                    selected = Some((start, length));
                }
            }
            selected.ok_or(Error::Unsupported)?
        };
        read(storage, start, &mut boot, progress)?;
        let geometry = Geometry::parse(&boot, start, limit)?;
        validate_boot(storage, start, progress)?;
        let mut backup = [0; 512];
        read(storage, start + 12, &mut backup, progress)?;
        if Geometry::parse(&backup, start, limit)? != geometry {
            return Err(Error::Corrupt);
        }
        validate_boot(storage, start + 12, progress)?;
        validate_chain(
            storage,
            geometry,
            geometry.root,
            None,
            MAX_ROOT_SECTORS,
            progress,
        )?;

        let mut bitmap = None;
        let mut upcase = None;
        let mut cursor = ChainCursor::new(geometry.root);
        let mut sector = [0; 512];
        let mut ended = false;
        while !ended {
            let Some(address) = cursor.next(storage, geometry, MAX_ROOT_SECTORS, progress)? else {
                break;
            };
            read(storage, address, &mut sector, progress)?;
            for entry in sector.as_chunks::<32>().0 {
                match entry[0] {
                    0 => {
                        ended = true;
                        break;
                    }
                    0x81 => {
                        if entry[1] & 1 != 0 || bitmap.is_some() {
                            return Err(Error::Corrupt);
                        }
                        bitmap = Some(Stream {
                            first: u32le(entry, 20),
                            bytes: u64le(entry, 24),
                        });
                    }
                    0x82 => {
                        if upcase.is_some() {
                            return Err(Error::Corrupt);
                        }
                        upcase = Some((
                            Stream {
                                first: u32le(entry, 20),
                                bytes: u64le(entry, 24),
                            },
                            u32le(entry, 4),
                        ));
                    }
                    _ => {}
                }
            }
        }
        let bitmap = bitmap.ok_or(Error::Corrupt)?;
        let (upcase, upcase_checksum) = upcase.ok_or(Error::Corrupt)?;
        if bitmap.bytes != u64::from(geometry.clusters).div_ceil(8)
            || bitmap.bytes.div_ceil(SECTOR) > u64::from(MAX_BITMAP_SECTORS)
            || upcase.bytes == 0
            || upcase.bytes > 131_072
            || !upcase.bytes.is_multiple_of(2)
        {
            return Err(Error::Unsupported);
        }
        for stream in [bitmap, upcase] {
            let count = stream
                .bytes
                .div_ceil(SECTOR * u64::from(geometry.sectors_per_cluster));
            validate_chain(
                storage,
                geometry,
                stream.first,
                Some(count as u32),
                MAX_BITMAP_SECTORS,
                progress,
            )?;
        }
        validate_upcase(storage, geometry, upcase, upcase_checksum, progress)?;
        let fs = Self {
            geometry,
            bitmap,
            writable: u16le(&boot, 106) & 6 == 0,
            active: None,
        };
        // Metadata must itself be allocated, otherwise an apparent free run
        // could overwrite the root, allocation map or character table.
        for stream in [
            Stream {
                first: geometry.root,
                bytes: 0,
            },
            bitmap,
            upcase,
        ] {
            fs.check_chain_allocated(storage, stream.first, progress)?;
        }
        fs.visit(storage, progress, &mut |_| {})?;
        Ok(fs)
    }

    pub const fn kind(&self) -> FileSystemKind {
        FileSystemKind::ExFat
    }
    pub const fn writable(&self) -> bool {
        self.writable
    }

    /// Visits only complete AE00000001.BIN-style files owned by this format.
    pub fn visit<B: BlockStorage>(
        &self,
        storage: &mut B,
        progress: &mut impl FnMut(),
        visitor: &mut impl FnMut(RecordingFile),
    ) -> Result<(), Error<B::Error>> {
        let mut entries = Entries::new(self.geometry.root);
        while let Some(entry) = entries.next(storage, self.geometry, progress)? {
            if entry[0] == 0 {
                break;
            }
            if entry[0] != 0x85 {
                if entry[0] & 0x80 != 0
                    && !matches!(entry[0], 0x81..=0x83)
                    && entry[0] & 0x20 == 0
                    && self.writable
                {
                    return Err(Error::Unsupported);
                }
                continue;
            }
            match self.read_file_set(storage, &mut entries, entry, progress) {
                Ok(Some(file)) => visitor(file),
                Ok(None) => {}
                // A new directory set may be incomplete after power loss.
                // Preserve read access to earlier independently validated
                // recordings; never repair or write an already dirty volume.
                Err(Error::Corrupt | Error::Unsupported | Error::Bounds) if !self.writable => {}
                Err(error) => return Err(error),
            }
        }
        Ok(())
    }

    fn read_file_set<B: BlockStorage>(
        &self,
        storage: &mut B,
        entries: &mut Entries,
        entry: [u8; 32],
        progress: &mut impl FnMut(),
    ) -> Result<Option<RecordingFile>, Error<B::Error>> {
        let count = usize::from(entry[1]);
        if count < 2 {
            return Err(Error::Corrupt);
        }
        let expected = u16le(&entry, 2);
        let mut checksum = set_checksum(0, &entry, true);
        let stream = entries
            .next(storage, self.geometry, progress)?
            .ok_or(Error::Corrupt)?;
        if stream[0] != 0xc0 {
            return Err(Error::Corrupt);
        }
        checksum = set_checksum(checksum, &stream, false);
        let mut name = [0_u16; 15];
        for index in 1..count {
            let next = entries
                .next(storage, self.geometry, progress)?
                .ok_or(Error::Corrupt)?;
            if next[0] & 0xc0 != 0xc0 {
                return Err(Error::Corrupt);
            }
            checksum = set_checksum(checksum, &next, false);
            if index == 1 && next[0] == 0xc1 {
                for (i, value) in name.iter_mut().enumerate() {
                    *value = u16le(&next, 2 + 2 * i);
                }
            }
        }
        if checksum != expected {
            return Err(Error::Corrupt);
        }
        if stream[3] != 14 {
            return Ok(None);
        }
        let Some(id) = recording_id(&name) else {
            return Ok(None);
        };
        if count != 2
            || u16le(&entry, 4) & 0x10 != 0
            || stream[1] & 3 != 3
            || name_hash(&name[..14]) != u16le(&stream, 4)
        {
            return Err(Error::Corrupt);
        }
        let length = u64le(&stream, 24);
        let first = u32le(&stream, 20);
        if length == 0
            || !length.is_multiple_of(SECTOR)
            || length != u64le(&stream, 8)
            || length / SECTOR > u64::from(u32::MAX)
        {
            return Err(Error::Corrupt);
        }
        let clusters = length.div_ceil(SECTOR * u64::from(self.geometry.sectors_per_cluster));
        if !self.geometry.valid_cluster(first)
            || u64::from(first) + clusters > u64::from(self.geometry.clusters) + 2
        {
            return Err(Error::Corrupt);
        }
        self.check_extent_allocated(storage, first, clusters as u32, progress)?;
        Ok(Some(self.file(id, first, (length / SECTOR) as u32)))
    }

    pub fn find<B: BlockStorage>(
        &self,
        storage: &mut B,
        id: u32,
        progress: &mut impl FnMut(),
    ) -> Result<Option<RecordingFile>, Error<B::Error>> {
        let mut found = None;
        let mut duplicate = false;
        self.visit(storage, progress, &mut |file| {
            if file.id == id {
                duplicate |= found.is_some();
                found = Some(file);
            }
        })?;
        if duplicate {
            return Err(Error::Corrupt);
        }
        Ok(found)
    }

    /// Reserve at most `target_sectors`, rounded down to whole clusters. If no
    /// run reaches that size, use the largest free run. A full/root-full card
    /// fails without mutation. Progress callbacks must not access this storage.
    pub fn create<B: BlockStorage>(
        &mut self,
        storage: &mut B,
        id: u32,
        target_sectors: u32,
        progress: &mut impl FnMut(),
    ) -> Result<RecordingFile, Error<B::Error>> {
        if !self.writable {
            return Err(Error::ReadOnly);
        }
        if id == 0 {
            return Err(Error::InvalidId);
        }
        if self.find(storage, id, progress)?.is_some() {
            return Err(Error::AlreadyExists);
        }
        let target = target_sectors / self.geometry.sectors_per_cluster;
        if target == 0 {
            return Err(Error::Bounds);
        }
        let slot = self.directory_slot(storage, progress)?;
        let (first, count) = self
            .free_run(storage, target, progress)?
            .ok_or(Error::NoSpace)?;
        let sectors = count
            .checked_mul(self.geometry.sectors_per_cluster)
            .ok_or(Error::Bounds)?;
        let file = self.file(id, first, sectors);
        // Fail closed even if the first write fails: its completion is unknown.
        self.writable = false;
        self.active = None;
        self.set_dirty(storage, true, progress)?;
        storage.flush().map_err(Error::Storage)?;
        self.allocate(storage, first, count, progress)?;
        storage.flush().map_err(Error::Storage)?;
        // A previous deleted file's first frame must not look like this session
        // before the new writer supplies its identifying CRC header.
        write(storage, file.first_sector, &[0; 512], progress)?;
        storage.flush().map_err(Error::Storage)?;
        self.publish_file(storage, slot, file, progress)?;
        storage.flush().map_err(Error::Storage)?;
        self.set_dirty(storage, false, progress)?;
        storage.flush().map_err(Error::Storage)?;
        self.writable = true;
        self.active = Some((file, 0));
        Ok(file)
    }

    pub fn read_sector<B: BlockStorage>(
        &self,
        storage: &mut B,
        file: &RecordingFile,
        relative_sector: u32,
        data: &mut [u8; 512],
    ) -> Result<(), Error<B::Error>> {
        let address = self.address(file, relative_sector)?;
        storage.read_sector(address, data).map_err(Error::Storage)
    }

    /// Read one to eight whole sectors from a validated contiguous extent.
    /// Bounds are checked for the complete request before any storage access.
    /// Reads never alter allocation, directory metadata or sequential-write state.
    pub fn read_sectors<B: ReadSectors>(
        &self,
        storage: &mut B,
        file: &RecordingFile,
        relative_sector: u32,
        data: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        if data.is_empty() || data.len() > 4096 || !data.len().is_multiple_of(512) {
            return Err(Error::Bounds);
        }
        let sectors = (data.len() / 512) as u32;
        let last = relative_sector
            .checked_add(sectors - 1)
            .ok_or(Error::Bounds)?;
        let address = self.address(file, relative_sector)?;
        self.address(file, last)?;
        storage.read_sectors(address, data).map_err(Error::Storage)
    }

    /// Only the newly created file accepts monotonically increasing offsets.
    /// Only the preallocated file's data is touched; no FAT/directory writes.
    pub fn write_sector<B: BlockStorage>(
        &mut self,
        storage: &mut B,
        file: &RecordingFile,
        relative_sector: u32,
        data: &[u8; 512],
    ) -> Result<(), Error<B::Error>> {
        if !self.writable {
            return Err(Error::ReadOnly);
        }
        if self.active != Some((*file, relative_sector)) {
            return Err(Error::Bounds);
        }
        let address = self.address(file, relative_sector)?;
        storage.write_sector(address, data).map_err(|error| {
            self.writable = false;
            Error::Storage(error)
        })?;
        self.active = Some((*file, relative_sector + 1));
        Ok(())
    }

    pub fn flush<B: BlockStorage>(&mut self, storage: &mut B) -> Result<(), Error<B::Error>> {
        storage.flush().map_err(|error| {
            self.writable = false;
            Error::Storage(error)
        })
    }

    fn file(&self, id: u32, first: u32, sectors: u32) -> RecordingFile {
        RecordingFile {
            id,
            first_sector: self.geometry.cluster_sector(first),
            sector_count: sectors,
            volume_start: self.geometry.start,
            volume_serial: self.geometry.serial,
        }
    }
    fn address<E>(&self, file: &RecordingFile, relative: u32) -> Result<u64, Error<E>> {
        if file.volume_start != self.geometry.start
            || file.volume_serial != self.geometry.serial
            || relative >= file.sector_count
            || file.first_sector < self.geometry.heap
            || file
                .first_sector
                .checked_add(u64::from(file.sector_count))
                .is_none_or(|end| end > self.geometry.start + self.geometry.length)
        {
            return Err(Error::Bounds);
        }
        Ok(file.first_sector + u64::from(relative))
    }

    fn set_dirty<B: BlockStorage>(
        &self,
        storage: &mut B,
        dirty: bool,
        progress: &mut impl FnMut(),
    ) -> Result<(), Error<B::Error>> {
        let mut boot = [0; 512];
        read(storage, self.geometry.start, &mut boot, progress)?;
        let flags = u16le(&boot, 106) & !0x0a;
        boot[106..108].copy_from_slice(&(flags | if dirty { 2 } else { 0 }).to_le_bytes());
        boot[112] = 0xff; // allocation percentage is now unknown
        write(storage, self.geometry.start, &boot, progress)
    }

    fn check_chain_allocated<B: BlockStorage>(
        &self,
        storage: &mut B,
        first: u32,
        progress: &mut impl FnMut(),
    ) -> Result<(), Error<B::Error>> {
        let mut cluster = first;
        for _ in 0..MAX_CHAIN_CLUSTERS {
            self.check_extent_allocated(storage, cluster, 1, progress)?;
            match next_cluster(storage, self.geometry, cluster, progress)? {
                Some(next) => cluster = next,
                None => return Ok(()),
            }
        }
        Err(Error::ScanLimit)
    }

    fn check_extent_allocated<B: BlockStorage>(
        &self,
        storage: &mut B,
        first: u32,
        count: u32,
        progress: &mut impl FnMut(),
    ) -> Result<(), Error<B::Error>> {
        let first_bit = u64::from(first - 2);
        let last_bit = first_bit + u64::from(count);
        let mut cursor = ChainCursor::new(self.bitmap.first);
        let mut data = [0; 512];
        let mut logical = 0_u64;
        while logical * 4096 < last_bit {
            let address = cursor
                .next(storage, self.geometry, MAX_BITMAP_SECTORS, progress)?
                .ok_or(Error::Corrupt)?;
            if (logical + 1) * 4096 > first_bit {
                read(storage, address, &mut data, progress)?;
                for bit in first_bit.max(logical * 4096)..last_bit.min((logical + 1) * 4096) {
                    let local = (bit % 4096) as usize;
                    if data[local / 8] & (1 << (local % 8)) == 0 {
                        return Err(Error::Corrupt);
                    }
                }
            }
            logical += 1;
        }
        Ok(())
    }

    fn free_run<B: BlockStorage>(
        &self,
        storage: &mut B,
        target: u32,
        progress: &mut impl FnMut(),
    ) -> Result<Option<(u32, u32)>, Error<B::Error>> {
        let mut cursor = ChainCursor::new(self.bitmap.first);
        let mut data = [0; 512];
        let mut bit = 0_u32;
        let mut run = 0_u32;
        let mut best = (0, 0);
        while bit < self.geometry.clusters {
            let address = cursor
                .next(storage, self.geometry, MAX_BITMAP_SECTORS, progress)?
                .ok_or(Error::Corrupt)?;
            read(storage, address, &mut data, progress)?;
            for byte in data {
                for shift in 0..8 {
                    if bit == self.geometry.clusters {
                        break;
                    }
                    if byte & (1 << shift) == 0 {
                        run += 1;
                        if run > best.1 {
                            best = (bit + 3 - run, run);
                        }
                        if run == target {
                            return Ok(Some(best));
                        }
                    } else {
                        run = 0;
                    }
                    bit += 1;
                }
            }
        }
        Ok((best.1 > 0).then_some(best))
    }

    fn allocate<B: BlockStorage>(
        &self,
        storage: &mut B,
        first: u32,
        count: u32,
        progress: &mut impl FnMut(),
    ) -> Result<(), Error<B::Error>> {
        let first_bit = u64::from(first - 2);
        let last_bit = first_bit + u64::from(count);
        let mut cursor = ChainCursor::new(self.bitmap.first);
        let mut data = [0; 512];
        let mut logical = 0_u64;
        while logical * 4096 < last_bit {
            let address = cursor
                .next(storage, self.geometry, MAX_BITMAP_SECTORS, progress)?
                .ok_or(Error::Corrupt)?;
            if (logical + 1) * 4096 > first_bit {
                read(storage, address, &mut data, progress)?;
                for bit in first_bit.max(logical * 4096)..last_bit.min((logical + 1) * 4096) {
                    let local = (bit % 4096) as usize;
                    let mask = 1 << (local % 8);
                    if data[local / 8] & mask != 0 {
                        return Err(Error::Corrupt);
                    }
                    data[local / 8] |= mask;
                }
                write(storage, address, &data, progress)?;
            }
            logical += 1;
        }
        Ok(())
    }

    fn directory_slot<B: BlockStorage>(
        &self,
        storage: &mut B,
        progress: &mut impl FnMut(),
    ) -> Result<DirectorySlot, Error<B::Error>> {
        let mut cursor = ChainCursor::new(self.geometry.root);
        let mut data = [0; 512];
        let mut bridge = None;
        while let Some(address) = cursor.next(storage, self.geometry, MAX_ROOT_SECTORS, progress)? {
            if bridge.is_some() {
                return Ok(DirectorySlot {
                    sector: address,
                    index: 0,
                    end: true,
                    bridge,
                });
            }
            read(storage, address, &mut data, progress)?;
            let mut unused = 0;
            for index in 0..16 {
                let kind = data[index * 32];
                if kind == 0 {
                    let start = index - unused;
                    if start <= 13 {
                        return Ok(DirectorySlot {
                            sector: address,
                            index: start as u8,
                            end: true,
                            bridge: None,
                        });
                    }
                    bridge = Some((address, index as u8));
                    break;
                }
                if kind & 0x80 == 0 {
                    unused += 1;
                } else {
                    unused = 0;
                }
                if unused == 3 {
                    return Ok(DirectorySlot {
                        sector: address,
                        index: (index - 2) as u8,
                        end: false,
                        bridge: None,
                    });
                }
            }
        }
        Err(Error::DirectoryFull)
    }

    fn publish_file<B: BlockStorage>(
        &self,
        storage: &mut B,
        slot: DirectorySlot,
        file: RecordingFile,
        progress: &mut impl FnMut(),
    ) -> Result<(), Error<B::Error>> {
        let mut sector = [0; 512];
        if let Some((previous, index)) = slot.bridge {
            // Establish the next terminator before removing the preceding one.
            write(storage, slot.sector, &sector, progress)?;
            storage.flush().map_err(Error::Storage)?;
            let mut tail = [0; 512];
            read(storage, previous, &mut tail, progress)?;
            for entry in tail[usize::from(index) * 32..].as_chunks_mut::<32>().0 {
                entry.fill(0);
                entry[0] = 1;
            }
            write(storage, previous, &tail, progress)?;
            storage.flush().map_err(Error::Storage)?;
        } else {
            read(storage, slot.sector, &mut sector, progress)?;
        }
        let offset = usize::from(slot.index) * 32;
        // If a new set consumes the end of this sector, preserve a terminator
        // in the next existing root sector before exposing that set.
        if slot.end && slot.index == 13 {
            let mut cursor = ChainCursor::new(self.geometry.root);
            while let Some(address) =
                cursor.next(storage, self.geometry, MAX_ROOT_SECTORS, progress)?
            {
                if address == slot.sector {
                    if let Some(next) =
                        cursor.next(storage, self.geometry, MAX_ROOT_SECTORS, progress)?
                    {
                        let mut following = [0; 512];
                        read(storage, next, &mut following, progress)?;
                        following[..32].fill(0);
                        write(storage, next, &following, progress)?;
                        storage.flush().map_err(Error::Storage)?;
                    }
                    break;
                }
            }
        }
        let set = directory_set(
            file.id,
            ((file.first_sector - self.geometry.heap)
                / u64::from(self.geometry.sectors_per_cluster)
                + 2) as u32,
            file.capacity_bytes(),
        );
        sector[offset..offset + 96].copy_from_slice(&set);
        if slot.end && offset + 96 < 512 {
            sector[offset + 96..offset + 128].fill(0);
        }
        write(storage, slot.sector, &sector, progress)
    }
}

#[derive(Clone, Copy)]
struct DirectorySlot {
    sector: u64,
    index: u8,
    end: bool,
    bridge: Option<(u64, u8)>,
}

impl Geometry {
    fn parse<E>(boot: &[u8; 512], start: u64, limit: u64) -> Result<Self, Error<E>> {
        if &boot[3..11] != b"EXFAT   "
            || boot[..3] != [0xeb, 0x76, 0x90]
            || u16le(boot, 510) != 0xaa55
            || boot[11..64].iter().any(|&v| v != 0)
        {
            return Err(Error::Corrupt);
        }
        if boot[108] != 9 || boot[109] > 16 || boot[110] != 1 || u16le(boot, 104) != 0x0100 {
            return Err(Error::Unsupported);
        }
        let length = u64le(boot, 72);
        let fat = u64::from(u32le(boot, 80));
        let fat_len = u64::from(u32le(boot, 84));
        let heap = u64::from(u32le(boot, 88));
        let clusters = u32le(boot, 92);
        let root = u32le(boot, 96);
        let sectors_per_cluster = 1_u32 << boot[109];
        if length < 2048
            || length > limit
            || (u64le(boot, 64) != 0 && u64le(boot, 64) != start)
            || fat < 24
            || fat_len < (u64::from(clusters) + 2).div_ceil(128)
            || fat + fat_len > heap
            || clusters == 0
            || clusters > 0xffff_fff5
            || heap + u64::from(clusters) * u64::from(sectors_per_cluster) > length
            || root < 2
            || root > clusters + 1
            || u16le(boot, 106) & 1 != 0
            || (boot[112] > 100 && boot[112] != 255)
        {
            return Err(Error::Corrupt);
        }
        Ok(Self {
            start,
            length,
            fat: start + fat,
            heap: start + heap,
            clusters,
            sectors_per_cluster,
            root,
            serial: u32le(boot, 100),
        })
    }
    fn valid_cluster(self, cluster: u32) -> bool {
        (2..=self.clusters + 1).contains(&cluster)
    }
    fn cluster_sector(self, cluster: u32) -> u64 {
        self.heap + u64::from(cluster - 2) * u64::from(self.sectors_per_cluster)
    }
}

struct ChainCursor {
    cluster: u32,
    within: u32,
    sectors: u32,
    ended: bool,
}
impl ChainCursor {
    fn new(first: u32) -> Self {
        Self {
            cluster: first,
            within: 0,
            sectors: 0,
            ended: false,
        }
    }
    fn next<B: BlockStorage>(
        &mut self,
        storage: &mut B,
        geometry: Geometry,
        limit: u32,
        progress: &mut impl FnMut(),
    ) -> Result<Option<u64>, Error<B::Error>> {
        if self.ended {
            return Ok(None);
        }
        if !geometry.valid_cluster(self.cluster) {
            return Err(Error::Corrupt);
        }
        if self.within == geometry.sectors_per_cluster {
            match next_cluster(storage, geometry, self.cluster, progress)? {
                Some(next) => {
                    self.cluster = next;
                    self.within = 0;
                }
                None => {
                    self.ended = true;
                    return Ok(None);
                }
            }
        }
        if self.sectors >= limit {
            return Err(Error::ScanLimit);
        }
        let address = geometry.cluster_sector(self.cluster) + u64::from(self.within);
        self.within += 1;
        self.sectors += 1;
        Ok(Some(address))
    }
}

struct Entries {
    cursor: ChainCursor,
    data: [u8; 512],
    index: usize,
}
impl Entries {
    fn new(first: u32) -> Self {
        Self {
            cursor: ChainCursor::new(first),
            data: [0; 512],
            index: 16,
        }
    }
    fn next<B: BlockStorage>(
        &mut self,
        storage: &mut B,
        geometry: Geometry,
        progress: &mut impl FnMut(),
    ) -> Result<Option<[u8; 32]>, Error<B::Error>> {
        if self.index == 16 {
            let Some(address) = self
                .cursor
                .next(storage, geometry, MAX_ROOT_SECTORS, progress)?
            else {
                return Ok(None);
            };
            read(storage, address, &mut self.data, progress)?;
            self.index = 0;
        }
        let mut result = [0; 32];
        result.copy_from_slice(&self.data[self.index * 32..(self.index + 1) * 32]);
        self.index += 1;
        Ok(Some(result))
    }
}

fn validate_chain<B: BlockStorage>(
    storage: &mut B,
    geometry: Geometry,
    first: u32,
    expected: Option<u32>,
    sector_limit: u32,
    progress: &mut impl FnMut(),
) -> Result<(), Error<B::Error>> {
    let mut current = first;
    let mut slow = Some(first);
    let mut fast = Some(first);
    for count in 1..=MAX_CHAIN_CLUSTERS {
        if !geometry.valid_cluster(current)
            || u64::from(count) * u64::from(geometry.sectors_per_cluster) > u64::from(sector_limit)
        {
            return Err(Error::ScanLimit);
        }
        let next = next_cluster(storage, geometry, current, progress)?;
        let Some(next) = next else {
            return if expected.is_none_or(|n| n == count) {
                Ok(())
            } else {
                Err(Error::Corrupt)
            };
        };
        if expected.is_some_and(|n| count >= n) {
            return Err(Error::Corrupt);
        }
        current = next;
        slow = match slow {
            Some(c) => next_cluster(storage, geometry, c, progress)?,
            None => None,
        };
        for _ in 0..2 {
            fast = match fast {
                Some(c) => next_cluster(storage, geometry, c, progress)?,
                None => None,
            };
        }
        if slow.is_some() && slow == fast {
            return Err(Error::Corrupt);
        }
    }
    Err(Error::ScanLimit)
}

fn next_cluster<B: BlockStorage>(
    storage: &mut B,
    geometry: Geometry,
    cluster: u32,
    progress: &mut impl FnMut(),
) -> Result<Option<u32>, Error<B::Error>> {
    if !geometry.valid_cluster(cluster) {
        return Err(Error::Corrupt);
    }
    let mut data = [0; 512];
    read(
        storage,
        geometry.fat + u64::from(cluster / 128),
        &mut data,
        progress,
    )?;
    let next = u32le(&data, (cluster % 128) as usize * 4);
    if next == 0xffff_ffff {
        Ok(None)
    } else if geometry.valid_cluster(next) {
        Ok(Some(next))
    } else {
        Err(Error::Corrupt)
    }
}

fn validate_boot<B: BlockStorage>(
    storage: &mut B,
    start: u64,
    progress: &mut impl FnMut(),
) -> Result<(), Error<B::Error>> {
    let mut checksum = 0_u32;
    let mut data = [0; 512];
    for sector in 0..11 {
        read(storage, start + sector, &mut data, progress)?;
        if (1..=8).contains(&sector) && u32le(&data, 508) != 0xaa55_0000 {
            return Err(Error::Corrupt);
        }
        for (index, &byte) in data.iter().enumerate() {
            if sector == 0 && matches!(index, 106 | 107 | 112) {
                continue;
            }
            checksum = checksum.rotate_right(1).wrapping_add(u32::from(byte));
        }
    }
    read(storage, start + 11, &mut data, progress)?;
    if data
        .as_chunks::<4>()
        .0
        .iter()
        .any(|word| u32le(word, 0) != checksum)
    {
        return Err(Error::Corrupt);
    }
    Ok(())
}

fn validate_upcase<B: BlockStorage>(
    storage: &mut B,
    geometry: Geometry,
    stream: Stream,
    expected: u32,
    progress: &mut impl FnMut(),
) -> Result<(), Error<B::Error>> {
    let mut cursor = ChainCursor::new(stream.first);
    let mut remaining = stream.bytes;
    let mut checksum = 0_u32;
    let mut codepoint = 0_u32;
    let mut run = false;
    let mut data = [0; 512];
    while remaining > 0 {
        let address = cursor
            .next(storage, geometry, MAX_BITMAP_SECTORS, progress)?
            .ok_or(Error::Corrupt)?;
        read(storage, address, &mut data, progress)?;
        let length = remaining.min(SECTOR) as usize;
        for &byte in &data[..length] {
            checksum = checksum.rotate_right(1).wrapping_add(u32::from(byte));
        }
        for word in data[..length].as_chunks::<2>().0 {
            let value = u16le(word, 0);
            if run {
                // Lowercase ASCII is never an identity mapping.
                if codepoint < 123 && codepoint + u32::from(value) > 97 {
                    return Err(Error::Unsupported);
                }
                codepoint += u32::from(value);
                run = false;
            } else if value == 0xffff && codepoint != 0xffff {
                run = true;
            } else {
                // Our names contain only uppercase ASCII, digits and '.'; their
                // mapping must be identity in the mounted volume's table.
                let expected = if (97..=122).contains(&codepoint) {
                    codepoint - 32
                } else {
                    codepoint
                };
                if codepoint < 128 && u32::from(value) != expected {
                    return Err(Error::Unsupported);
                }
                codepoint += 1;
            }
            if codepoint > 65_536 {
                return Err(Error::Corrupt);
            }
        }
        remaining -= length as u64;
    }
    if checksum != expected || run || codepoint != 65_536 {
        return Err(Error::Corrupt);
    }
    Ok(())
}

fn directory_set(id: u32, first: u32, length: u64) -> [u8; 96] {
    let mut set = [0; 96];
    set[0] = 0x85;
    set[1] = 2;
    set[4] = 0x20; // archive, regular file
    // 1980-01-01, unspecified UTC offset. The log contains precise GNSS/capture time.
    for offset in [8, 12, 16] {
        set[offset..offset + 4].copy_from_slice(&0x0021_0000_u32.to_le_bytes());
    }
    set[32] = 0xc0;
    set[33] = 3; // AllocationPossible | NoFatChain
    set[35] = 14;
    set[40..48].copy_from_slice(&length.to_le_bytes());
    set[52..56].copy_from_slice(&first.to_le_bytes());
    set[56..64].copy_from_slice(&length.to_le_bytes());
    set[64] = 0xc1;
    let mut name = [0_u16; 14];
    name[0] = b'A' as u16;
    name[1] = b'E' as u16;
    for index in 0..8 {
        name[index + 2] = u16::from(b"0123456789ABCDEF"[((id >> ((7 - index) * 4)) & 15) as usize]);
    }
    for (index, byte) in b".BIN".iter().enumerate() {
        name[index + 10] = u16::from(*byte);
    }
    set[36..38].copy_from_slice(&name_hash(&name).to_le_bytes());
    for (index, value) in name.iter().enumerate() {
        set[66 + index * 2..68 + index * 2].copy_from_slice(&value.to_le_bytes());
    }
    let checksum = set_checksum(0, &set, true);
    set[2..4].copy_from_slice(&checksum.to_le_bytes());
    set
}

fn recording_id(name: &[u16; 15]) -> Option<u32> {
    let name = name.map(ascii_upper);
    if name[..2] != [b'A' as u16, b'E' as u16]
        || name[10..14] != [b'.' as u16, b'B' as u16, b'I' as u16, b'N' as u16]
    {
        return None;
    }
    let mut id = 0_u32;
    for &value in &name[2..10] {
        let digit = match value {
            48..=57 => value - 48,
            65..=70 => value - 65 + 10,
            _ => return None,
        };
        id = (id << 4) | u32::from(digit);
    }
    (id != 0).then_some(id)
}
fn name_hash(name: &[u16]) -> u16 {
    let mut hash = 0_u16;
    for &value in name {
        for byte in ascii_upper(value).to_le_bytes() {
            hash = hash.rotate_right(1).wrapping_add(u16::from(byte));
        }
    }
    hash
}
fn ascii_upper(value: u16) -> u16 {
    if (97..=122).contains(&value) {
        value - 32
    } else {
        value
    }
}
fn set_checksum(mut checksum: u16, data: &[u8], primary: bool) -> u16 {
    for (index, &byte) in data.iter().enumerate() {
        if primary && matches!(index, 2 | 3) {
            continue;
        }
        checksum = checksum.rotate_right(1).wrapping_add(u16::from(byte));
    }
    checksum
}
fn read<B: BlockStorage>(
    storage: &mut B,
    address: u64,
    data: &mut [u8; 512],
    progress: &mut impl FnMut(),
) -> Result<(), Error<B::Error>> {
    if address >= storage.sector_count() {
        return Err(Error::Bounds);
    }
    let result = storage.read_sector(address, data).map_err(Error::Storage);
    progress();
    result
}
fn write<B: BlockStorage>(
    storage: &mut B,
    address: u64,
    data: &[u8; 512],
    progress: &mut impl FnMut(),
) -> Result<(), Error<B::Error>> {
    if address >= storage.sector_count() {
        return Err(Error::Bounds);
    }
    let result = storage.write_sector(address, data).map_err(Error::Storage);
    progress();
    result
}
fn u16le(data: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([data[offset], data[offset + 1]])
}
fn u32le(data: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap())
}
fn u64le(data: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(data[offset..offset + 8].try_into().unwrap())
}

#[cfg(test)]
#[path = "filesystem_tests.rs"]
mod tests;
