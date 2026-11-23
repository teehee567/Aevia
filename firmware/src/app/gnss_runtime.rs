//! SPP/PPS decoding remains integer-only in the UART interrupt executor. Raw
//! COM1 bytes are buffered for ordinary PPK decoding; all FP publication is normal.
use super::*;
use crate::{drivers::um980, timing, uart_pipe::Port};
use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, select};

fn gap(port: Port) {
    UART_PIPE.lock(|pipe| pipe.borrow_mut().reset_port(port));
    GNSS_VERIFICATION_INVALIDATE.store(true, Ordering::Release);
    timing::configuration_verified(false);
    UART_READABLE.signal(());
}

fn receive_data(decoder: &mut timing::wire::Decoder<256>, bytes: &[u8], received_us: u64) {
    use timing::wire::Frame;
    for byte in bytes {
        let native = match decoder.push(*byte) {
            Ok(Some(Frame::SppNav(native))) => native,
            Ok(Some(Frame::Pps(label))) => {
                if timing::is_configuration_verified() {
                    if let Some(now_ns) = crate::capture::now_ns() {
                        timing::observe_label(label, now_ns);
                    } else {
                        timing::invalidate();
                    }
                }
                continue;
            }
            Ok(Some(Frame::InvalidPps)) => {
                timing::invalidate();
                continue;
            }
            Ok(Some(Frame::InvalidSppNav)) | Err(_) => {
                update(|state| state.gnss.errors = state.gnss.errors.saturating_add(1));
                timing::invalidate();
                continue;
            }
            Ok(Some(Frame::Line(_))) | Ok(None) => continue,
        };
        GNSS_NATIVE.lock(|state| {
            *state.borrow_mut() = Some(GnssNativeStatus {
                received_us,
                time_reference: native.time_reference,
                time_status: native.time_status,
                gps_week: native.gps_week,
                gps_tow_ms: native.gps_tow_ms,
                position_status: native.position_status(),
                position_type: native.position_type(),
                velocity_status: native.velocity_status(),
                velocity_type: native.velocity_type(),
                version: native.version,
                output_delay_ms: native.output_delay_ms,
            })
        });
        if native.time_reference != 0 || native.gps_tow_ms >= 604_800_000 {
            update(|state| state.gnss.errors = state.gnss.errors.saturating_add(1));
            timing::invalidate();
            continue;
        }
        let mapping = native
            .qualified_epoch_ms()
            .and_then(|_| crate::capture::now_ns())
            .and_then(|now| timing::map_epoch(u32::from(native.gps_week), native.gps_tow_ms, now));
        let sequence = GNSS_OBSERVATION_DRAIN.lock(|drain| drain.borrow().next_sequence());
        let observation = GnssObservation {
            sequence,
            received_us,
            native,
            mapping,
        };
        if GNSS_OBSERVATIONS.try_send(observation).is_ok() {
            GNSS_OBSERVATION_DRAIN.lock(|drain| drain.borrow_mut().produced(sequence));
        } else {
            GNSS_UART.lock(|state| {
                let mut state = state.borrow_mut();
                state.queue_dropped = state.queue_dropped.saturating_add(1);
            });
            update(|state| state.gnss.errors = state.gnss.errors.saturating_add(1));
            if STATE.lock(|state| state.borrow().recording.active) || stop_pending() {
                crate::storage::note_gnss_observation_loss();
            }
        }
        UART_READABLE.signal(());
    }
}

fn transport_error(port: Port, reason: &'static str, now: u64, dropped_bytes: Option<usize>) {
    GNSS_UART.lock(|state| {
        let mut state = state.borrow_mut();
        state.errors = state.errors.saturating_add(1);
        state.last_error = Some(reason);
        state.last_port = Some(if port == Port::Data {
            "data"
        } else {
            "command"
        });
        state.last_error_us = Some(now);
    });
    update(|s| {
        s.gnss.errors = s.gnss.errors.saturating_add(1);
        s.gnss.status = reason;
    });
    if STATE.lock(|state| state.borrow().recording.active) || stop_pending() {
        crate::storage::note_gnss_transport_loss(port, dropped_bytes);
    }
}

#[embassy_executor::task]
pub async fn gnss_acquire(command: Uart<'static, Blocking>, data: Uart<'static, Blocking>) {
    let mut command = command.into_async();
    let mut data = data.into_async();
    loop {
        GNSS_RECEIVING.store(false, Ordering::Release);
        gap(Port::Data);
        gap(Port::Command);
        update(|s| s.gnss.status = "configuring");
        PPK.lock(|state| state.borrow_mut().configured = false);
        if let Err(error) = um980::configure(&mut command, &mut data).await {
            update(|s| {
                s.gnss.status = error;
                s.gnss.errors = s.gnss.errors.saturating_add(1);
            });
            Timer::after_secs(3).await;
            continue;
        }
        let configured_at = Instant::now().as_micros();
        GNSS_RECEIVING.store(true, Ordering::Release);
        UART_READABLE.signal(());
        let mut data_bytes = [0; 128];
        let mut command_bytes = [0; 128];
        let mut data_decoder = timing::wire::Decoder::<256>::new();
        let mut last_data_read = None;
        let mut last_short_command_read = None;
        let mut prefer_data = true;
        let mut outgoing = um980::PPK_OBSERVATIONS;
        let mut next_ephemeris = um980::PPK_EPHEMERIDES.len();
        let mut ephemeris_due = Instant::now();
        let mut restoring = false;
        let mut restore_done = None;
        PPK_EPHEMERIDES.lock(|request| *request.borrow_mut() = true);
        loop {
            if restore_done.is_some_and(|at| Instant::now() >= at) {
                RECEIVER_RESTORE.lock(|state| *state.borrow_mut() = 2);
                restore_done = None;
            }
            if outgoing.is_empty()
                && !restoring
                && RECEIVER_RESTORE.lock(|state| *state.borrow() == 1)
            {
                outgoing = b"UNLOG COM1\r\nCONFIG COM1 115200\r\n";
                restoring = true;
            }
            if outgoing.is_empty() && !restoring && GNSS_CONFIG_QUERY.swap(false, Ordering::AcqRel)
            {
                outgoing = b"CONFIG\r\n";
            }
            if outgoing.is_empty()
                && !restoring
                && PPK_EPHEMERIDES
                    .lock(|request| core::mem::replace(&mut *request.borrow_mut(), false))
            {
                next_ephemeris = 0;
                ephemeris_due = Instant::now();
            }
            // A simultaneous dump of every constellation can occupy COM1 long
            // enough to starve publication of native IMU intervals at Start.
            if outgoing.is_empty()
                && !restoring
                && Instant::now() >= ephemeris_due
                && let Some(pair) = um980::PPK_EPHEMERIDES.get(next_ephemeris)
            {
                outgoing = pair;
                next_ephemeris += 1;
                ephemeris_due = Instant::now() + Duration::from_millis(800);
            }
            if !outgoing.is_empty() && command.write_ready() {
                match command.write(outgoing) {
                    Ok(written) => {
                        outgoing = &outgoing[written..];
                        if outgoing.is_empty() && restoring {
                            restore_done = Some(Instant::now() + Duration::from_millis(20));
                        }
                    }
                    Err(_) => {
                        outgoing = &[];
                        PPK.lock(|state| {
                            let mut state = state.borrow_mut();
                            state.command_errors = state.command_errors.saturating_add(1);
                            state.configured = false;
                        });
                        gap(Port::Command);
                    }
                }
            }
            let incoming = with_timeout(
                Duration::from_millis(if outgoing.is_empty() { 100 } else { 1 }),
                async {
                    if prefer_data {
                        match select(
                            embedded_io_async::Read::read(&mut data, &mut data_bytes),
                            embedded_io_async::Read::read(&mut command, &mut command_bytes),
                        )
                        .await
                        {
                            Either::First(result) => (Port::Data, result),
                            Either::Second(result) => (Port::Command, result),
                        }
                    } else {
                        match select(
                            embedded_io_async::Read::read(&mut command, &mut command_bytes),
                            embedded_io_async::Read::read(&mut data, &mut data_bytes),
                        )
                        .await
                        {
                            Either::First(result) => (Port::Command, result),
                            Either::Second(result) => (Port::Data, result),
                        }
                    }
                },
            )
            .await;
            let began = Instant::now();
            let received_us = began.as_micros();
            prefer_data = !prefer_data;
            let mut coalesce_command = false;
            match incoming {
                Ok((port, Ok(length))) => {
                    RUNTIME_TIMING.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.gnss_reads = state.gnss_reads.saturating_add(1);
                        state.gnss_zero_reads =
                            state.gnss_zero_reads.saturating_add(u32::from(length == 0));
                        if port == Port::Data {
                            state.gnss_data_bytes =
                                state.gnss_data_bytes.saturating_add(length as u64);
                        } else {
                            state.gnss_raw_bytes =
                                state.gnss_raw_bytes.saturating_add(length as u64);
                        }
                    });
                    if length != 0 && port == Port::Data {
                        if last_data_read
                            .is_some_and(|last| received_us.saturating_sub(last) > 100_000)
                        {
                            data_decoder.reset();
                        }
                        last_data_read = Some(received_us);
                        receive_data(&mut data_decoder, &data_bytes[..length], received_us);
                    } else if length != 0 {
                        let previous_short_read = last_short_command_read.take();
                        if length < 16 {
                            last_short_command_read = Some(received_us);
                            coalesce_command = previous_short_read
                                .is_some_and(|previous| received_us.saturating_sub(previous) < 250);
                        }
                        let queued = UART_PIPE.lock(|pipe| {
                            pipe.borrow_mut().try_push(
                                port,
                                received_us,
                                None,
                                &command_bytes[..length],
                            )
                        });
                        match queued {
                            Ok(sequence) => {
                                GNSS_DRAIN.lock(|drain| drain.borrow_mut().produced(sequence))
                            }
                            Err(()) => {
                                transport_error(
                                    port,
                                    "byte-queue-overflow",
                                    received_us,
                                    Some(length),
                                );
                                GNSS_VERIFICATION_INVALIDATE.store(true, Ordering::Release);
                                timing::configuration_verified(false);
                            }
                        }
                        UART_READABLE.signal(());
                    }
                }
                Ok((port, Err(error))) => {
                    let reason = match error {
                        esp_hal::uart::IoError::Rx(esp_hal::uart::RxError::FifoOverflowed) => {
                            "fifo-overflow"
                        }
                        esp_hal::uart::IoError::Rx(esp_hal::uart::RxError::GlitchOccurred) => {
                            "glitch"
                        }
                        esp_hal::uart::IoError::Rx(esp_hal::uart::RxError::FrameFormatViolated) => {
                            "frame-format"
                        }
                        esp_hal::uart::IoError::Rx(esp_hal::uart::RxError::ParityMismatch) => {
                            "parity"
                        }
                        _ => "uart-read-error",
                    };
                    transport_error(port, reason, received_us, None);
                    if port == Port::Data {
                        data_decoder.reset();
                    }
                    gap(port);
                }
                Err(_) => {}
            }
            RUNTIME_TIMING.lock(|state| {
                let mut state = state.borrow_mut();
                state.gnss_io_work_us = state
                    .gnss_io_work_us
                    .saturating_add(began.elapsed().as_micros());
            });
            if coalesce_command {
                // Continuous receiver replies can keep this interrupt executor
                // runnable on only a few bytes per read. Give the ordinary IMU
                // and GNSS consumers a turn and let the UART collect a larger
                // batch. A 250 µs pause allows about 12 bytes to arrive at
                // 460800 baud. Isolated short replies are published immediately.
                Timer::after_micros(250).await;
            }
            let last = STATE
                .lock(|state| state.borrow().gnss.last_us)
                .max(configured_at);
            if !restoring && received_us.saturating_sub(last) > 3_000_000 {
                update(|s| s.gnss.status = "no-valid-records");
                break;
            }
        }
    }
}

struct Decoder {
    command: timing::wire::Decoder<1024>,
    raw: crate::ppk::Decoder,
    verification: timing::Verification,
    generations: [u32; 2],
    last_read: [Option<u64>; 2],
}
impl Decoder {
    fn new() -> Self {
        Self {
            command: timing::wire::Decoder::new(),
            raw: crate::ppk::Decoder::new(),
            verification: timing::Verification::default(),
            generations: [0; 2],
            last_read: [None; 2],
        }
    }
    fn reset_decoder(&mut self, port: Port) {
        if port == Port::Command {
            self.command.reset();
            self.raw.reset();
        }
        self.last_read[port.index()] = None;
    }
    fn reset(&mut self, port: Port) {
        self.reset_decoder(port);
        self.verification.failed(Instant::now().as_micros());
        timing::invalidate();
    }
    fn publish_state(&self) {
        PPK.lock(|state| {
            let mut state = state.borrow_mut();
            let diagnostics = self.raw.diagnostics();
            if GNSS_RECEIVING.load(Ordering::Acquire)
                && diagnostics.observation_frames > state.diagnostics.observation_frames
            {
                state.configured = true;
            }
            state.diagnostics = diagnostics;
            state.in_flight_received_us = self.raw.inflight_received_us();
        });
        UART_PIPE.lock(|pipe| {
            // Revoke in IRQ immediately; an older queued CONFIG response must
            // not re-enable PPS between a loss and its decoder reset.
            let current = self.generations == pipe.borrow().generations()
                && GNSS_RECEIVING.load(Ordering::Acquire)
                && !GNSS_VERIFICATION_INVALIDATE.load(Ordering::Acquire);
            timing::configuration_verified(current && self.verification.verified());
        });
    }
    fn maintain(&mut self, empty_generations: Option<[u32; 2]>) {
        let now = Instant::now().as_micros();
        if GNSS_VERIFICATION_INVALIDATE.swap(false, Ordering::AcqRel) {
            self.verification.failed(now);
        }
        if !GNSS_RECEIVING.load(Ordering::Acquire) {
            self.verification = timing::Verification::default();
            timing::invalidate();
        } else {
            match self.verification.poll(now) {
                timing::Action::Query => {
                    GNSS_CONFIG_QUERY.store(true, Ordering::Release);
                }
                timing::Action::TimedOut => timing::invalidate(),
                timing::Action::None => {}
            }
        }
        if let Some(generations) = empty_generations {
            for port in [Port::Data, Port::Command] {
                let index = port.index();
                if self.generations[index] != generations[index] {
                    self.reset(port);
                    self.generations[index] = generations[index];
                } else if self.last_read[index].is_some_and(|at| now.saturating_sub(at) > 100_000) {
                    // The queue was drained before deciding that the wire went
                    // idle; ordinary-context scheduling latency is not a gap.
                    self.reset_decoder(port);
                }
            }
            if self
                .raw
                .inflight_received_us()
                .is_some_and(|at| now.saturating_sub(at) > 2_000_000)
            {
                self.reset(Port::Command);
            }
        }
        self.publish_state();
    }
    fn process(&mut self, batch: &crate::uart_pipe::ReadBatch) {
        use timing::wire::Frame;
        let port = batch.port;
        let index = port.index();
        if self.generations[index] != batch.generation {
            self.reset(port);
            self.generations[index] = batch.generation;
        } else if self.last_read[index]
            .is_some_and(|previous| batch.received_us.saturating_sub(previous) > 100_000)
        {
            // A naturally quiet raw port between 5 Hz observations is not a
            // configuration or transport failure.
            self.reset_decoder(port);
        }
        if port == Port::Command
            && self
                .raw
                .inflight_received_us()
                .is_some_and(|at| batch.received_us.saturating_sub(at) > 2_000_000)
        {
            self.reset_decoder(port);
        }
        self.last_read[index] = Some(batch.received_us);
        for byte in &batch.bytes[..batch.len] {
            if port == Port::Command
                && let Some(chunk) = self.raw.push(*byte, batch.received_us)
                && STATE.lock(|state| state.borrow().recording.contains(chunk.received_us()))
            {
                crate::storage::capture_raw(chunk);
            }
            let line = match self.command.push(*byte) {
                Ok(Some(Frame::Line(line))) => line,
                Ok(Some(Frame::Pps(_) | Frame::SppNav(_) | Frame::InvalidSppNav)) => continue,
                Ok(Some(Frame::InvalidPps)) | Err(_) => {
                    timing::invalidate();
                    continue;
                }
                Ok(None) => continue,
            };
            if GNSS_RECEIVING.load(Ordering::Acquire) {
                let verified = self.verification.verified();
                self.verification.observe(line, Instant::now().as_micros());
                if verified && !self.verification.verified() {
                    timing::invalidate();
                }
            }
            if line.starts_with(b"$command,") {
                for expected in um980::PPK_EPHEMERIDES
                    .iter()
                    .flat_map(|pair| pair.split(|b| *b == b'\n'))
                    .chain(core::iter::once(um980::PPK_OBSERVATIONS))
                {
                    let expected = expected.strip_suffix(b"\n").unwrap_or(expected);
                    let expected = expected.strip_suffix(b"\r").unwrap_or(expected);
                    if crate::gnss::parse_command_ack(line, expected) == Some(false) {
                        PPK.lock(|state| {
                            let mut state = state.borrow_mut();
                            state.command_errors = state.command_errors.saturating_add(1);
                            state.configured = false;
                        });
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn gnss() {
    let mut decoder = Decoder::new();
    let mut processed = 0;
    let mut last_yield = Instant::now();
    loop {
        for _ in 0..4 {
            let Ok(observation) = GNSS_OBSERVATIONS.try_receive() else {
                break;
            };
            let sequence = observation.sequence;
            publish(observation);
            GNSS_OBSERVATION_DRAIN.lock(|drain| drain.borrow_mut().consumed(sequence));
        }
        let (batch, empty_generations) = UART_PIPE.lock(|pipe| {
            let mut pipe = pipe.borrow_mut();
            let batch = pipe.pop();
            let generations = batch.is_none().then(|| pipe.generations());
            (batch, generations)
        });
        let Some(batch) = batch else {
            decoder.maintain(empty_generations);
            maybe_close_storage();
            if GNSS_OBSERVATIONS.is_empty() {
                let _ = with_timeout(Duration::from_millis(100), UART_READABLE.wait()).await;
            } else {
                embassy_futures::yield_now().await;
            }
            processed = 0;
            last_yield = Instant::now();
            continue;
        };
        let began = Instant::now();
        decoder.process(&batch);
        GNSS_DRAIN.lock(|drain| drain.borrow_mut().consumed(batch.sequence));
        RUNTIME_TIMING.lock(|state| {
            let mut state = state.borrow_mut();
            let elapsed = began.elapsed().as_micros();
            state.gnss_work_us = state.gnss_work_us.saturating_add(elapsed);
            state.gnss_work_max_us = state
                .gnss_work_max_us
                .max(elapsed.min(u64::from(u32::MAX)) as u32);
        });
        processed += batch.len;
        if processed >= 512 || last_yield.elapsed().as_micros() >= 2_000 {
            // No ordinary task can observe the decoder between these reads.
            // Publish its complete state before yielding to a possible Stop.
            decoder.maintain(None);
            maybe_close_storage();
            embassy_futures::yield_now().await;
            processed = 0;
            last_yield = Instant::now();
        }
    }
}

fn publish(observation: GnssObservation) {
    let native = observation.native;
    let measurement = match native.measurement() {
        Ok(measurement) => measurement,
        Err(_) => {
            update(|state| state.gnss.errors = state.gnss.errors.saturating_add(1));
            None
        }
    };
    if let Some(owned) = measurement
        .as_ref()
        .and_then(|value| value.navigation(&native))
    {
        crate::navigation::capture_gnss(
            owned,
            observation.received_us,
            observation.mapping.map(|e| e.time_ns / 1000),
        );
    }
    let nav = measurement.as_ref();
    let mapping = observation.mapping;
    enqueue(Sample::Gnss {
        t_us: observation.received_us,
        gps_week: u32::from(native.gps_week),
        gps_tow_ms: native.gps_tow_ms,
        lat: nav.map_or(0.0, |n| n.latitude_deg),
        lon: nav.map_or(0.0, |n| n.longitude_deg),
        alt_m: nav.map_or(0.0, |n| n.height_m + n.undulation_m),
        speed_mps: nav.map_or(0.0, |n| n.horizontal_speed_m_s),
        heading_deg: nav.map_or(0.0, |n| n.track_deg),
        vertical_mps: nav.map_or(0.0, |n| n.vertical_speed_m_s),
        hacc_m: nav.map_or(0.0, |n| n.latitude_sigma_m.max(n.longitude_sigma_m)),
        vacc_m: nav.map_or(0.0, |n| n.height_sigma_m),
        sacc_mps: nav.map_or(0.0, |n| n.horizontal_speed_sigma_m_s),
        latency_ms: nav.map_or(0, |n| libm::round(n.latency_s * 1000.0) as u32),
        satellites: native.satellites_used(),
        valid: nav.is_some(),
        mapped_ns: mapping.map(|m| m.time_ns),
        mapping_uncertainty_ns: mapping.map(|m| m.uncertainty_ns),
        mapping_generation: mapping.map(|m| m.generation),
    });
    if !GNSS_RECEIVING.load(Ordering::Acquire) {
        update(|s| s.gnss.status = "configuring");
    }
}
