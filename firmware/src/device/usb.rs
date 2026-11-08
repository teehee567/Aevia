use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{CdcAcmClass, State},
};
use esp_hal::{
    peripherals::USB_HS,
    usb::otg::{Usb, embassy_usb_device::Driver},
};

pub type Device<'d> = UsbDevice<'d, Driver<'d>>;
pub type Serial<'d> = CdcAcmClass<'d, Driver<'d>>;

pub struct Buffers<'d> {
    out: [u8; 1024],
    descriptors: [u8; 256],
    bos: [u8; 256],
    control: [u8; 64],
    serial_number: [u8; 12],
    state: State<'d>,
}

impl Buffers<'_> {
    pub const fn new() -> Self {
        Self {
            out: [0; 1024],
            descriptors: [0; 256],
            bos: [0; 256],
            control: [0; 64],
            serial_number: [0; 12],
            state: State::new(),
        }
    }
}

impl Default for Buffers<'_> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn init<'d>(peripheral: USB_HS<'d>, buffers: &'d mut Buffers<'d>) -> (Device<'d>, Serial<'d>) {
    let driver = Driver::new(
        Usb::new_hs(peripheral),
        &mut buffers.out,
        Default::default(),
    );

    // Development identity, separate from the ROM downloader's 303a:0020.
    let mut config = Config::new(0x303a, 0x4001);
    config.manufacturer = Some("Aevia");
    config.product = Some("Aevia Loader");
    let mac = esp_hal::efuse::base_mac_address();
    for (index, byte) in mac.as_bytes().iter().copied().enumerate() {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        buffers.serial_number[index * 2] = HEX[(byte >> 4) as usize];
        buffers.serial_number[index * 2 + 1] = HEX[(byte & 15) as usize];
    }
    config.serial_number = core::str::from_utf8(&buffers.serial_number).ok();

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
