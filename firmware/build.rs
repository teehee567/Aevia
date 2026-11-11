use sha2::{Digest, Sha256};
use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn sources(dir: &Path, paths: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(dir).expect("source directory") {
        let path = entry.expect("source entry").path();
        if path.is_dir() {
            if !matches!(
                path.file_name().and_then(|s| s.to_str()),
                Some("target" | ".git")
            ) {
                sources(&path, paths);
            }
        } else if path.extension().is_some_and(|s| s == "rs") {
            paths.push(path);
        }
    }
}

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let mut paths = Vec::new();
    for directory in ["src", "../traj/src"] {
        let path = root.join(directory);
        println!("cargo:rerun-if-changed={}", path.display());
        sources(&path, &mut paths);
    }
    for file in [
        "build.rs",
        "Cargo.toml",
        ".cargo/config.toml",
        "src/device/hal/Cargo.toml",
        "../Cargo.toml",
        "../Cargo.lock",
        "../traj/Cargo.toml",
    ] {
        let path = root.join(file);
        println!("cargo:rerun-if-changed={}", path.display());
        paths.push(path);
    }
    paths.sort();
    let mut hash = Sha256::new();
    for path in paths {
        let relative = path
            .strip_prefix(&root)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        let data = fs::read(&path).expect("source read");
        hash.update((relative.len() as u64).to_le_bytes());
        hash.update(relative.as_bytes());
        hash.update((data.len() as u64).to_le_bytes());
        hash.update(data);
    }
    println!("cargo:rustc-env=AEVIA_BUILD_ID={:x}", hash.finalize());
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("riscv32") {
        println!("cargo:rustc-link-arg=-Tlinkall.x");
    }
}
