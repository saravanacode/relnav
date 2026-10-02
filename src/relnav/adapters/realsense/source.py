"""RealSense frame source: a capture thread fills a small newest-first queue."""

# pyrealsense2 is a compiled module without type stubs.
# pyright: reportAttributeAccessIssue=false

from __future__ import annotations

import contextlib
import queue
import threading
import time
from collections.abc import Callable

import numpy as np
import pyrealsense2 as rs
from numpy.typing import NDArray

from relnav.adapters.realsense.config import RealSenseConfig
from relnav.domain.types import Frame, Intrinsics
from relnav.ports.errors import FrameSourceError, FrameTimeout


def frame_from_arrays(
    index: int,
    timestamp_ns: int,
    ir: NDArray[np.uint8],
    depth_units: NDArray[np.uint16],
    intr: Intrinsics,
) -> Frame:
    depth_m = depth_units.astype(np.float32) * np.float32(intr.depth_scale)
    return Frame(index, timestamp_ns, np.array(ir, dtype=np.uint8, copy=True), depth_m, intr)


def start_with_retry[T](
    start: Callable[[], T],
    attempts: int,
    delay_s: float,
    sleep: Callable[[float], None] = time.sleep,
) -> T:
    """macOS sometimes refuses the first USB claim ("failed to set power state")."""
    last_error: Exception | None = None
    for attempt in range(1, attempts + 1):
        try:
            return start()
        except RuntimeError as exc:
            last_error = exc
            if attempt < attempts:
                sleep(delay_s)
    raise FrameSourceError(f"camera did not start after {attempts} attempts: {last_error}")


class LatestFrameQueue:
    """Bounded queue that drops the oldest frame when processing falls behind."""

    def __init__(self, maxsize: int) -> None:
        self._queue: queue.Queue[Frame] = queue.Queue(maxsize=maxsize)
        self.dropped = 0

    def put(self, frame: Frame) -> None:
        while True:
            try:
                self._queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def get(self, timeout_s: float) -> Frame | None:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None


class RealSenseFrameSource:
    def __init__(self, cfg: RealSenseConfig | None = None) -> None:
        self._cfg = cfg = cfg or RealSenseConfig()
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
        config.enable_stream(rs.stream.infrared, 1, cfg.width, cfg.height, rs.format.y8, cfg.fps)
        profile = start_with_retry(
            lambda: self._pipeline.start(config), cfg.start_attempts, cfg.start_retry_delay_s
        )
        try:
            sensor = profile.get_device().first_depth_sensor()
            if sensor.supports(rs.option.emitter_enabled):
                sensor.set_option(rs.option.emitter_enabled, 1.0 if cfg.emitter_enabled else 0.0)
            stream = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            self.intrinsics = Intrinsics(
                width=stream.width,
                height=stream.height,
                fx=stream.fx,
                fy=stream.fy,
                cx=stream.ppx,
                cy=stream.ppy,
                depth_scale=sensor.get_depth_scale(),
            )
        except RuntimeError as exc:  # device lost between start and setup
            with contextlib.suppress(RuntimeError):
                self._pipeline.stop()
            raise FrameSourceError(f"camera setup failed: {exc}") from exc
        # No temporal filter: it smears depth while the camera moves.
        self._filters = [rs.threshold_filter(cfg.min_depth_m, cfg.max_depth_m), rs.spatial_filter()]
        self._queue = LatestFrameQueue(cfg.queue_size)
        self._stop = threading.Event()
        self._error: str | None = None
        self._thread = threading.Thread(target=self._capture, name="realsense", daemon=True)
        self._thread.start()

    @property
    def dropped_frames(self) -> int:
        return self._queue.dropped

    def next_frame(self, timeout_s: float) -> Frame | None:
        frame = self._queue.get(timeout_s)
        if frame is not None:
            return frame
        if self._error is not None:
            raise FrameSourceError(self._error)
        raise FrameTimeout(f"no frame within {timeout_s} s")

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)
        with contextlib.suppress(RuntimeError):
            self._pipeline.stop()

    def _capture(self) -> None:
        index, last_timestamp, misses = 0, -1, 0
        while not self._stop.is_set():
            try:
                frames = self._pipeline.wait_for_frames(1000)
            except RuntimeError as exc:
                misses += 1
                if misses >= self._cfg.max_missed_waits:
                    self._error = f"camera stopped delivering frames: {exc}"
                    return
                continue
            misses = 0
            depth, ir = frames.get_depth_frame(), frames.get_infrared_frame(1)
            if not depth or not ir:
                continue
            for f in self._filters:
                depth = f.process(depth)
            timestamp = max(int(frames.get_timestamp() * 1e6), last_timestamp + 1)
            last_timestamp = timestamp
            self._queue.put(
                frame_from_arrays(
                    index,
                    timestamp,
                    np.asanyarray(ir.get_data()),
                    np.asanyarray(depth.get_data()),
                    self.intrinsics,
                )
            )
            index += 1
