use super::update::{APP_SIZE, CHUNK, Error, Flash, Updater};
use crate::commands::{Command, Response};

pub fn command<F: Flash>(
    updater: &mut Updater<F>,
    command: Command<'_>,
) -> Result<Response, Error> {
    let result = execute(updater, command);
    if result.is_err() {
        updater.abort();
    }
    result
}

fn execute<F: Flash>(updater: &mut Updater<F>, command: Command<'_>) -> Result<Response, Error> {
    match command {
        Command::Info => {
            return Ok(Response::Info {
                recovery: true,
                max_size: APP_SIZE,
                chunk: CHUNK as u16,
            });
        }
        Command::Abort => updater.abort(),
        Command::Run => {
            updater.request_app()?;
            return Ok(Response::Reboot);
        }
        Command::Begin { size, hash } => updater.begin(size, hash)?,
        Command::Data { offset, bytes } => updater.data(offset, bytes)?,
        Command::Commit => {
            updater.commit()?;
            return Ok(Response::Reboot);
        }
        Command::Recover => return Err(Error::Command),
        Command::BootStatus => return Ok(Response::BootStatus { stage: 0 }),
        Command::ProtocolInfo => return Ok(Response::ProtocolInfo(crate::protocol::info(true))),
    }
    Ok(Response::Ok)
}
