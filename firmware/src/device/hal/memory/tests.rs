use super::*;
use embedded_storage::nor_flash::{ErrorType, ReadNorFlash};
use std::{
    cell::{Cell, RefCell},
    rc::Rc,
    vec,
    vec::Vec,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Injected {
    Transport,
}
impl NorFlashError for Injected {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}
#[derive(Clone, Debug, Eq, PartialEq)]
enum Op {
    Read(u32, usize),
    Program(u32, Vec<u8>),
    Erase(u32, u32),
}
struct Fake<const R: usize = 2, const W: usize = 4, const E: usize = 16> {
    bytes: Vec<u8>,
    calls: Rc<RefCell<Vec<Op>>>,
    fail: Rc<Cell<bool>>,
}
impl<const R: usize, const W: usize, const E: usize> Fake<R, W, E> {
    fn new(capacity: usize) -> Self {
        Self {
            bytes: vec![0xff; capacity],
            calls: Rc::default(),
            fail: Rc::default(),
        }
    }
    fn result(&self) -> Result<(), Injected> {
        if self.fail.get() {
            Err(Injected::Transport)
        } else {
            Ok(())
        }
    }
}
impl<const R: usize, const W: usize, const E: usize> ErrorType for Fake<R, W, E> {
    type Error = Injected;
}
impl<const R: usize, const W: usize, const E: usize> ReadNorFlash for Fake<R, W, E> {
    const READ_SIZE: usize = R;
    fn capacity(&self) -> usize {
        self.bytes.len()
    }
    fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), Self::Error> {
        self.calls.borrow_mut().push(Op::Read(address, data.len()));
        self.result()?;
        data.copy_from_slice(&self.bytes[address as usize..address as usize + data.len()]);
        Ok(())
    }
}
impl<const R: usize, const W: usize, const E: usize> NorFlash for Fake<R, W, E> {
    const WRITE_SIZE: usize = W;
    const ERASE_SIZE: usize = E;
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.calls
            .borrow_mut()
            .push(Op::Program(address, data.to_vec()));
        self.result()?;
        for (dest, source) in self.bytes[address as usize..].iter_mut().zip(data) {
            *dest &= *source;
        }
        Ok(())
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.calls.borrow_mut().push(Op::Erase(from, to));
        self.result()?;
        self.bytes[from as usize..to as usize].fill(0xff);
        Ok(())
    }
}
fn fake() -> Fake {
    Fake::new(64)
}

#[test]
fn explicit_region_is_inert_and_translates_all_operation_addresses() {
    let backend = fake();
    let calls = backend.calls.clone();
    let mut view = NorRegion::new(backend, 16, 32).unwrap();
    assert!(calls.borrow().is_empty());
    assert_eq!(
        (
            view.base_offset_bytes(),
            view.capacity_bytes(),
            view.read_size_bytes(),
            view.program_size_bytes(),
            view.erase_size_bytes()
        ),
        (16, 32, 2, 4, 16)
    );
    view.program(4, &[1, 2, 3, 4]).unwrap();
    let mut bytes = [0; 4];
    view.read(4, &mut bytes).unwrap();
    assert_eq!(bytes, [1, 2, 3, 4]);
    view.erase(16, 16).unwrap();
    assert_eq!(
        *calls.borrow(),
        [
            Op::Program(20, vec![1, 2, 3, 4]),
            Op::Read(20, 4),
            Op::Erase(32, 48)
        ]
    );
}

#[test]
fn erase_never_rounds_into_neighboring_regions_and_program_never_erases() {
    let mut backend = fake();
    backend.bytes[..16].fill(0xa5);
    backend.bytes[16..48].fill(0);
    backend.bytes[48..].fill(0x5a);
    let mut view = NorRegion::new(backend, 16, 32).unwrap();
    view.erase(0, 16).unwrap();
    view.program(0, &[0x12; 4]).unwrap();
    let backend = view.release();
    assert_eq!(&backend.bytes[..16], &[0xa5; 16]);
    assert_eq!(&backend.bytes[16..20], &[0x12; 4]);
    assert_eq!(&backend.bytes[20..32], &[0xff; 12]);
    assert_eq!(&backend.bytes[32..48], &[0; 16]);
    assert_eq!(&backend.bytes[48..], &[0x5a; 16]);
    assert_eq!(
        *backend.calls.borrow(),
        [Op::Erase(16, 32), Op::Program(16, vec![0x12; 4])]
    );
}

#[test]
fn invalid_region_geometry_and_ranges_never_call_backend() {
    let mut backend = fake();
    assert!(matches!(
        NorRegion::new(&mut backend, 1, 32),
        Err(Error::Unaligned)
    ));
    assert!(matches!(
        NorRegion::new(&mut backend, 16, 15),
        Err(Error::Unaligned)
    ));
    assert!(matches!(
        NorRegion::new(&mut backend, 16, 64),
        Err(Error::OutOfBounds)
    ));
    assert!(matches!(
        NorRegion::new(&mut backend, u64::MAX, 2),
        Err(Error::OutOfBounds)
    ));
    assert!(backend.calls.borrow().is_empty());
    assert!(matches!(
        NorRegion::new(Fake::<0, 4, 16>::new(64), 0, 32),
        Err(Error::InvalidGeometry)
    ));
    assert!(matches!(
        NorRegion::new(Fake::<2, 0, 16>::new(64), 0, 32),
        Err(Error::InvalidGeometry)
    ));
    assert!(matches!(
        NorRegion::new(Fake::<2, 4, 0>::new(64), 0, 32),
        Err(Error::InvalidGeometry)
    ));
}

#[test]
fn operation_alignment_and_overflow_reject_before_transport() {
    let backend = fake();
    let calls = backend.calls.clone();
    let mut view = NorRegion::new(backend, 16, 32).unwrap();
    assert_eq!(view.read(1, &mut [0; 2]), Err(Error::Unaligned));
    assert_eq!(view.read(0, &mut [0; 3]), Err(Error::Unaligned));
    assert_eq!(view.program(2, &[0; 4]), Err(Error::Unaligned));
    assert_eq!(view.program(0, &[0; 3]), Err(Error::Unaligned));
    assert_eq!(view.erase(1, 16), Err(Error::Unaligned));
    assert_eq!(view.erase(0, 15), Err(Error::Unaligned));
    assert_eq!(view.erase(16, 32), Err(Error::OutOfBounds));
    assert_eq!(view.program(u64::MAX, &[0; 4]), Err(Error::OutOfBounds));
    assert_eq!(view.erase(u64::MAX, 4), Err(Error::OutOfBounds));
    assert_eq!(view.read(32, &mut [0; 2]), Err(Error::OutOfBounds));
    assert!(calls.borrow().is_empty());
}

#[test]
fn empty_operations_are_silent_but_still_constrained_to_region() {
    let backend = fake();
    let calls = backend.calls.clone();
    let mut view = NorRegion::new(backend, 16, 32).unwrap();
    view.read(1, &mut []).unwrap();
    view.program(32, &[]).unwrap();
    view.erase(32, 0).unwrap();
    assert_eq!(view.read(33, &mut []), Err(Error::OutOfBounds));
    assert_eq!(view.erase(33, 0), Err(Error::OutOfBounds));
    assert!(calls.borrow().is_empty());
}

#[test]
fn backend_errors_are_preserved_without_retry_or_extra_erase() {
    let backend = fake();
    let calls = backend.calls.clone();
    backend.fail.set(true);
    let mut view = NorRegion::new(backend, 16, 32).unwrap();
    assert_eq!(
        view.program(0, &[0; 4]),
        Err(Error::Backend(Injected::Transport))
    );
    assert_eq!(
        view.read(0, &mut [0; 2]),
        Err(Error::Backend(Injected::Transport))
    );
    assert_eq!(view.erase(0, 16), Err(Error::Backend(Injected::Transport)));
    assert_eq!(
        *calls.borrow(),
        [
            Op::Program(16, vec![0; 4]),
            Op::Read(16, 2),
            Op::Erase(16, 32)
        ]
    );
    assert_eq!(
        Error::Backend(Injected::Transport).kind(),
        NorFlashErrorKind::Other
    );
}

struct MetadataOnly;
impl ErrorType for MetadataOnly {
    type Error = Infallible;
}
impl ReadNorFlash for MetadataOnly {
    const READ_SIZE: usize = 1;
    fn capacity(&self) -> usize {
        usize::MAX
    }
    fn read(&mut self, _: u32, _: &mut [u8]) -> Result<(), Self::Error> {
        panic!("unexpected read");
    }
}
impl NorFlash for MetadataOnly {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;
    fn write(&mut self, _: u32, _: &[u8]) -> Result<(), Self::Error> {
        panic!("unexpected write");
    }
    fn erase(&mut self, _: u32, _: u32) -> Result<(), Self::Error> {
        panic!("unexpected erase");
    }
}
#[test]
fn u32_exclusive_end_is_never_truncated() {
    assert!(matches!(
        NorRegion::new(MetadataOnly, 0xffff_f000, 4096),
        Err(Error::AddressWidth)
    ));
}

#[test]
fn mini_profile_checks_real_capacity_and_keeps_range_explicit() {
    assert!(matches!(
        NorRegion::v2_mini(fake(), 0, 32),
        Err(Error::CapacityMismatch {
            expected: V2_MINI_FLASH_BYTES,
            actual: 64
        })
    ));
    assert_eq!(
        Part::Esp32S31N16R16VFlash.verified_capacity_bytes(),
        Some(V2_MINI_FLASH_BYTES)
    );
    assert_eq!(
        Part::InfineonS80ks5122Gabhv020.verified_capacity_bytes(),
        None
    );
    assert_eq!(Part::ApMemoryAps512xxn.verified_capacity_bytes(), None);
}

#[test]
fn ram_views_preserve_existing_bytes_and_bound_subwindows() {
    let mut data = [0xa5; 16];
    let mut ram = RamSlice::new(&mut data);
    let mut before = [0; 16];
    ram.read(0, &mut before).unwrap();
    assert_eq!(before, [0xa5; 16]);
    {
        let mut window = ram.window(4, 4).unwrap();
        assert_eq!(window.capacity_bytes(), 4);
        window.write(0, &[1, 2, 3, 4]).unwrap();
        assert_eq!(window.write(4, &[9]), Err(Error::OutOfBounds));
    }
    assert_eq!(
        ram.release(),
        &[
            0xa5, 0xa5, 0xa5, 0xa5, 1, 2, 3, 4, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5
        ]
    );
}

#[test]
fn ram_overflow_and_out_of_range_do_not_partially_write() {
    let mut data = [0x55; 8];
    let mut ram = RamSlice::new(&mut data);
    assert_eq!(ram.write(7, &[1, 2]), Err(Error::OutOfBounds));
    assert_eq!(ram.write(usize::MAX, &[1]), Err(Error::OutOfBounds));
    assert_eq!(ram.read(usize::MAX, &mut [0; 2]), Err(Error::OutOfBounds));
    assert!(matches!(ram.window(usize::MAX, 2), Err(Error::OutOfBounds)));
    ram.write(8, &[]).unwrap();
    ram.read(8, &mut []).unwrap();
    assert_eq!(ram.write(9, &[]), Err(Error::OutOfBounds));
    assert_eq!(ram.release(), &[0x55; 8]);
}

#[test]
fn common_storage_traits_forward_exact_bounds_and_contents() {
    let mut flash = NorRegion::new(fake(), 16, 32).unwrap();
    let storage: &mut dyn crate::interfaces::NorStorage<Error = Error<Injected>> = &mut flash;
    assert_eq!(storage.capacity_bytes(), 32);
    storage.program(0, &[1, 2, 3, 4]).unwrap();
    let mut data = [0; 4];
    storage.read(0, &mut data).unwrap();
    assert_eq!(data, [1, 2, 3, 4]);
    let mut bytes = [0; 4];
    let mut ram = RamSlice::new(&mut bytes);
    let storage: &mut dyn crate::interfaces::RamStorage<Error = Error<Infallible>> = &mut ram;
    storage.write(0, &data).unwrap();
    data.fill(0);
    storage.read(0, &mut data).unwrap();
    assert_eq!(data, [1, 2, 3, 4]);
}
