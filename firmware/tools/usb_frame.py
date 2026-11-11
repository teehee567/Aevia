"""Bounded zero-terminated replies for a USB client with one request outstanding."""
import time


class FrameError(RuntimeError):
    pass


class FrameReader:
    """Drain available bytes in bulk, blocking only while waiting for new bytes.

    The short serial timeout is set once, not once per byte or per response. A
    separate deadline bounds the whole frame even if a broken device dribbles
    bytes indefinitely. This reader deliberately rejects trailing data: the
    request/reply protocol permits exactly one response per outstanding request.
    """

    def __init__(self, device, max_bytes=512, timeout=3):
        if not isinstance(max_bytes, int) or max_bytes < 1 or not 0 < timeout < float("inf"):
            raise ValueError("frame bound and timeout must be positive and finite")
        self.device = device
        self.max_bytes = max_bytes
        self.timeout = timeout
        self.read_timeout = min(0.05, timeout)
        self.device.timeout = self.read_timeout

    def read_frame(self):
        deadline = time.monotonic() + self.timeout
        frame = bytearray()
        adjusted_timeout = False
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise FrameError("USB response interrupted or timed out; reconnect before retrying")
                waiting = self.device.in_waiting
                # read(large_bound) would wait for absent bytes until timeout;
                # request only bytes already buffered, or one blocking byte.
                count = min(waiting or 1, self.max_bytes + 1 - len(frame))
                if not waiting and remaining < self.read_timeout:
                    self.device.timeout = remaining
                    adjusted_timeout = True
                value = self.device.read(count)
                if time.monotonic() > deadline:
                    raise FrameError("USB response interrupted or timed out; reconnect before retrying")
                frame.extend(value)
                delimiter = frame.find(0)
                if delimiter >= 0:
                    if delimiter + 1 > self.max_bytes:
                        raise FrameError("USB response oversized; reconnect before retrying")
                    if delimiter + 1 != len(frame):
                        raise FrameError("trailing USB response data; reconnect before retrying")
                    return bytes(frame)
                if len(frame) >= self.max_bytes:
                    raise FrameError("USB response oversized; reconnect before retrying")
        finally:
            if adjusted_timeout:
                self.device.timeout = self.read_timeout
