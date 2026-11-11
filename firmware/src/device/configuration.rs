//! Build-time accessory selection shared by board startup and protocol discovery.
//! No automatic accessory detection or persistent configuration is implied.

pub const DISPLAY_FITTED: bool = false;
// Connected evaluation board has a user-supplied 1S, 3.7 V, 2000 mAh battery.
pub const BATTERY_FITTED: bool = true;
/// Initial supervised profile, not a cell-manufacturer qualification. The PCB
/// thermistor is retained; it does not independently establish cell temperature.
pub const CHARGING_ENABLED: bool = true;
pub const CHARGE_VOLTAGE_MV: u16 = 4_100;
// User-requested setting, exactly representable in the charger's 80 mA steps.
pub const CHARGE_CURRENT_MA: u16 = 800;
pub const PRECHARGE_CURRENT_MA: u16 = 80;
pub const TERMINATION_CURRENT_MA: u16 = 40;
pub const BUTTONS_FITTED: bool = false;
/// Race firmware reserves CPU1 for persistence; offline processing consumes the
/// original measurements. Acquisition and USB diagnostics remain available.
pub const DATA_COLLECTION_ONLY: bool = true;
/// SHA256 of firmware/trajectory source and build inputs, distinct from the
/// final image SHA256 in the release manifest.
pub const BUILD_ID: &str = env!("AEVIA_BUILD_ID");
