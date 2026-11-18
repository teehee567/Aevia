/// Executes a native device, recording, library or analysis operation.
/// The bridge runs this on its worker pool rather than Flutter's UI thread.
pub fn request(request_json: String) -> String {
    crate::service::request(request_json)
}

#[flutter_rust_bridge::frb(init)]
pub fn init_app() {
    flutter_rust_bridge::setup_default_user_utils();
}
