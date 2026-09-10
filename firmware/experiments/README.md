# Archived bring-up experiments

These files preserve existing work and hardware evidence. Cargo does not
compile this directory. They are reference source, not runnable firmware targets.

- `trajectory.rs`: previous onboard trajectory adapter. The maintained engine
  remains in `traj/`; reintroduce an application integration through an explicit
  task and memory owner when needed.
- `power_diagnostic.rs`: former startup snapshot. The live battery monitor now
  owns charger/gauge reads, and board initialization owns I2C timeouts.

Historical behavior and results are in `../docs/trajectory-poc.md` and
`../docs/history/`. Current base firmware is documented in `../README.md`.
