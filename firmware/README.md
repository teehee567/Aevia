# V2 Mini firmware

From this directory, build and upload the application:

```powershell
cargo run --release --locked
```

Firmware targets only `riscv32imafc-unknown-none-elf`.
Requires that Rust target, `uv`, and `espflash`.
The single uploader, `tools/upload.py`, accepts the Cargo ELF or an application
`.bin`. It enters recovery over USB, uploads, and waits for application USB.
Its only Python dependency is pyserial, installed automatically by uv.

```powershell
uv run --script tools/upload.py --recover
uv run --script tools/upload.py target/upload/firmware.bin
```

The loader verifies SHA-256 from flash before booting. Interrupted uploads,
panics, and the ten-second watchdog return to recovery. USB and watchdog
handling run independently of `src/app.rs`.

## Initial recovery installation

Enter ROM mode on J6: hold SW2, tap SW1, release SW2. Build both binaries,
then run the following from the repository root in the S31 ESP-IDF environment.
Replace COM3 with the ROM serial port.

```powershell
cargo build --manifest-path firmware/Cargo.toml --target riscv32imafc-unknown-none-elf --release --bins --locked
idf.py -C firmware/bootloader -B target/current-bootloader '-DIDF_TARGET=esp32s31' bootloader
espflash save-image --chip esp32s31 --flash-size 16mb --flash-mode dio --flash-freq 40mhz --merge --skip-padding --bootloader target/current-bootloader/bootloader/bootloader.bin --partition-table firmware/partitions.csv --target-app-partition recovery target/riscv32imafc-unknown-none-elf/release/recovery target/loader-install.bin
python -m esptool --chip esp32s31 --port COM3 --before no-reset --after watchdog-reset --no-stub write-flash --flash-mode dio --flash-freq 40m --flash-size 16MB 0 target/loader-install.bin
```

Subsequent application uploads use `cargo run` without buttons.
Preserve `target/bootloader-backup-before-flash.bin` if present.

## Checks

From this directory:

```powershell
cargo build --release --bins --locked
cargo clippy --release --bins --locked -- -D warnings
uv run --script tools/upload.py --help
uv run --with pyserial==3.5 python -m unittest discover -s tools
```

## Layout and protocol

`partitions.csv` reserves state at `0x9000`, recovery at `0x10000..0x90000`,
and the application at `0x90000..0x1000000`. SDK crates come from crates.io;
S31 flash-capacity and watchdog fixes live in `src/flash.rs` and
`src/watchdog.rs`.

USB IDs are `C0C0:CAFE` for recovery and `C0C0:FEED` for the application.
`src/loader.rs` defines postcard/COBS commands: `Recover`,
`Upload { size: u32, sha256: [u8; 32] }`, and `Data(&[u8])`.
Replies are `Ready`, `Reboot`, or `Error`. Send one command per reply,
with 256-byte data blocks and a final short block. Frames end in zero and
partial requests time out after 30 seconds.
