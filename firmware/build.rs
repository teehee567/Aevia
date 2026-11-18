fn main() {
    let output = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    std::fs::write(
        output.join("aevia-stack.x"),
        include_str!("stack-reserve.x"),
    )
    .unwrap();
    println!("cargo:rerun-if-changed=stack-reserve.x");
    println!("cargo:rustc-link-search={}", output.display());
    println!("cargo:rustc-link-arg=-Tlinkall.x");
    println!("cargo:rustc-link-arg=-Taevia-stack.x");
}
