use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use esp_hal::{
    peripherals::USB_HS,
    usb::otg::{Usb, embassy_usb_device::Driver},
};
use static_cell::StaticCell;

pub type Device = UsbDevice<'static, Driver<'static>>;
pub type Serial = CdcAcmClass<'static, Driver<'static>>;
pub type Input = Receiver<'static, Driver<'static>>;
pub type Output = Sender<'static, Driver<'static>>;
pub const PACKET_SIZE: usize = 512;

static BUFFERS: StaticCell<Buffers> = StaticCell::new();

struct Buffers {
    out: [u8; 1024],
    descriptors: [u8; 256],
    bos: [u8; 256],
    control: [u8; 64],
    state: State<'static>,
}

pub fn init(peripheral: USB_HS<'static>, recovery: bool) -> (Device, Serial) {
    let buffers = BUFFERS.init(Buffers {
        out: [0; 1024],
        descriptors: [0; 256],
        bos: [0; 256],
        control: [0; 64],
        state: State::new(),
    });
    let driver = Driver::new(
        Usb::new_hs(peripheral),
        &mut buffers.out,
        Default::default(),
    );
    let mut config = Config::new(0xc0c0, if recovery { 0xcafe } else { 0xfeed });
    config.manufacturer = Some("Aevia");
    config.product = Some(if recovery { "Aevia Loader" } else { "Aevia" });
    config.serial_number = Some("AEVIADEVBOARD");
    let mut builder = Builder::new(
        driver,
        config,
        &mut buffers.descriptors,
        &mut buffers.bos,
        &mut [],
        &mut buffers.control,
    );
    let serial = CdcAcmClass::new(&mut builder, &mut buffers.state, PACKET_SIZE as u16);
    (builder.build(), serial)
}
