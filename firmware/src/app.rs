#[embassy_executor::task]
pub async fn run() {
    core::future::pending::<()>().await;
}
