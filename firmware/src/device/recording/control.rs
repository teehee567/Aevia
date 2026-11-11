use super::api::{Command, Error, Reply, Request, Response, Status};

/// Exactly one storage operation in flight; eight completed replies retained.
/// The monotonic token is consumed on admission, never on execution. Evicted
/// requests cannot execute again, even after later Start/Stop operations.
pub struct Mailbox {
    next: u32,
    pending: Option<Request>,
    running: Option<Request>,
    cache: [Option<(Request, Reply)>; 8],
    cursor: usize,
}
impl Mailbox {
    pub const fn new() -> Self {
        Self {
            next: 1,
            pending: None,
            running: None,
            cache: [None; 8],
            cursor: 0,
        }
    }
    pub fn request(&mut self, request: Request, mut status: Status) -> Response {
        let reply = if request.command == Command::Status {
            status.next_request_id = self.next;
            Reply::Status(status)
        } else if let Some((old, reply)) = self
            .cache
            .iter()
            .flatten()
            .find(|(old, _)| old.request_id == request.request_id)
        {
            if request.command == Command::Result || request.command == old.command {
                *reply
            } else {
                Reply::Error(Error::RequestConflict)
            }
        } else if let Some(old) = self
            .pending
            .or(self.running)
            .filter(|r| r.request_id == request.request_id)
        {
            if request.command == Command::Result || request.command == old.command {
                Reply::Pending
            } else {
                Reply::Error(Error::RequestConflict)
            }
        } else if request.command == Command::Result
            || request.request_id < self.next
            || self.next == 0
        {
            Reply::Error(Error::UnknownRequest)
        } else if request.request_id != self.next {
            Reply::Error(Error::RequestConflict)
        } else if self.pending.is_some() || self.running.is_some() {
            Reply::Error(Error::Busy)
        } else {
            self.next = self.next.checked_add(1).unwrap_or(0);
            self.pending = Some(request);
            Reply::Pending
        };
        Response {
            request_id: request.request_id,
            reply,
        }
    }
    pub fn take(&mut self) -> Option<Request> {
        if self.running.is_some() {
            return None;
        }
        let request = self.pending.take()?;
        self.running = Some(request);
        Some(request)
    }
    pub fn complete(&mut self, request: Request, reply: Reply) {
        assert_eq!(self.running, Some(request));
        self.cache[self.cursor] = Some((request, reply));
        self.cursor = (self.cursor + 1) % self.cache.len();
        self.running = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn request(id: u32, command: Command) -> Request {
        Request {
            request_id: id,
            command,
        }
    }
    #[test]
    fn duplicate_conflict_and_evicted_mutations_never_repeat() {
        let mut m = Mailbox::new();
        let status = Status::default();
        let first = request(1, Command::Start);
        assert_eq!(m.request(first, status).reply, Reply::Pending);
        assert_eq!(m.request(first, status).reply, Reply::Pending);
        assert_eq!(
            m.request(request(1, Command::Stop), status).reply,
            Reply::Error(Error::RequestConflict)
        );
        assert_eq!(
            m.request(request(2, Command::Stop), status).reply,
            Reply::Error(Error::Busy)
        );
        assert_eq!(m.take(), Some(first));
        assert!(m.take().is_none());
        m.complete(first, Reply::Error(Error::NoCard));
        assert_eq!(m.request(first, status).reply, Reply::Error(Error::NoCard));
        for id in 2..=10 {
            let next = request(id, Command::Stop);
            assert_eq!(m.request(next, status).reply, Reply::Pending);
            assert_eq!(m.take(), Some(next));
            m.complete(next, Reply::Error(Error::NotRecording));
        }
        assert_eq!(
            m.request(first, status).reply,
            Reply::Error(Error::UnknownRequest)
        );
        assert_eq!(
            m.request(request(1, Command::Result), status).reply,
            Reply::Error(Error::UnknownRequest)
        );
        let Reply::Status(s) = m.request(request(0, Command::Status), status).reply else {
            panic!()
        };
        assert_eq!(s.next_request_id, 11);
    }
    #[test]
    fn exhausted_token_space_never_wraps_and_replays_old_start() {
        let mut m = Mailbox::new();
        m.next = u32::MAX;
        let r = request(u32::MAX, Command::Start);
        assert_eq!(m.request(r, Status::default()).reply, Reply::Pending);
        m.take();
        m.complete(r, Reply::Error(Error::NoCard));
        assert_eq!(
            m.request(request(1, Command::Start), Status::default())
                .reply,
            Reply::Error(Error::UnknownRequest)
        );
    }
}
