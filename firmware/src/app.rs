/// Application code starts here. Recovery never starts this task.
#[embassy_executor::task]
pub async fn run() {
    core::future::pending::<()>().await;
}
