use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{CdcAcmClass, State},
};
use esp_hal::{
    peripherals::USB_HS,
    usb::otg::{Usb, embassy_usb_device::Driver},
};

use crate::globals::{DEVICE_PID, LOADER_PID, SERIALID, USBVID};

pub type Device<'d> = UsbDevice<'d, Driver<'d>>;
pub type Serial<'d> = CdcAcmClass<'d, Driver<'d>>;

pub struct Buffers<'d> {
    out: [u8; 1024],
    descriptors: [u8; 256],
    bos: [u8; 256],
    control: [u8; 64],
    msos: [u8; 0],
    state: State<'d>,
}

impl Buffers<'_> {
    pub const fn new() -> Self {
        Self {
            out: [0; 1024],
            descriptors: [0; 256],
            bos: [0; 256],
            control: [0; 64],
            msos: [],
            state: State::new(),
        }
    }
}

impl Default for Buffers<'_> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn init<'d>(
    peripheral: USB_HS<'d>,
    buffers: &'d mut Buffers<'d>,
    recovery: bool,
) -> (Device<'d>, Serial<'d>) {
    let driver = Driver::new(
        Usb::new_hs(peripheral),
        &mut buffers.out,
        Default::default(),
    );

    let mut config = Config::new(USBVID, if recovery { LOADER_PID } else { DEVICE_PID });
    config.manufacturer = Some("teehee567");
    config.product = Some(if recovery { "Aevia Loader" } else { "Aevia" });
    config.serial_number = Some(SERIALID);

    let mut builder = Builder::new(
        driver,
        config,
        &mut buffers.descriptors,
        &mut buffers.bos,
        &mut buffers.msos,
        &mut buffers.control,
    );
    let serial = CdcAcmClass::new(&mut builder, &mut buffers.state, 512);
    (builder.build(), serial)
}
