use std::env;

fn main() {
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("riscv32") {
        println!("cargo:rustc-link-arg=-Tlinkall.x");
    }
}
