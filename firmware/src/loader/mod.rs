#[cfg(any(target_arch = "riscv32", test))]
pub(crate) mod layout;
pub mod recovery;
pub mod state;
pub mod update;

#[cfg(target_arch = "riscv32")]
pub mod boot;
#[cfg(test)]
mod tests;
