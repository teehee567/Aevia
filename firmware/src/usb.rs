use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{BufferedReceiver, CdcAcmClass, Sender, State},
};
use esp_hal::{
    peripherals::USB_HS,
    usb::otg::{Usb, embassy_usb_device::Driver},
};

pub type Device = UsbDevice<'static, Driver<'static>>;
pub type Serial = CdcAcmClass<'static, Driver<'static>>;
pub type Input = BufferedReceiver<'static, Driver<'static>>;
pub type Output = Sender<'static, Driver<'static>>;

pub struct Buffers {
    out: [u8; 1024],
    descriptors: [u8; 256],
    bos: [u8; 256],
    control: [u8; 64],
    state: State<'static>,
}

impl Default for Buffers {
    fn default() -> Self {
        Self {
            out: [0; 1024],
            descriptors: [0; 256],
            bos: [0; 256],
            control: [0; 64],
            state: State::new(),
        }
    }
}

pub fn init(peripheral: USB_HS<'static>, buffers: &'static mut Buffers) -> (Device, Serial) {
    let driver = Driver::new(
        Usb::new_hs(peripheral),
        &mut buffers.out,
        Default::default(),
    );
    let mut config = Config::new(0xc0c0, 0xcafe);
    config.manufacturer = Some("Aevia");
    config.product = Some("Aevia");
    config.serial_number = Some("AEVIADEVBOARD");
    let mut builder = Builder::new(
        driver,
        config,
        &mut buffers.descriptors,
        &mut buffers.bos,
        &mut buffers.control,
        &mut [],
    );
    let serial = CdcAcmClass::new(&mut builder, &mut buffers.state, 512);
    (builder.build(), serial)
}
