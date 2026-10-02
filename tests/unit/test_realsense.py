import numpy as np
import pytest

from relnav.adapters.realsense.config import RealSenseConfig
from relnav.adapters.realsense.source import LatestFrameQueue, frame_from_arrays, start_with_retry
from relnav.domain.types import Frame, Intrinsics
from relnav.ports.errors import FrameSourceError

INTR = Intrinsics(4, 3, 2.0, 2.0, 2.0, 1.5, depth_scale=0.001)


def frame(i: int) -> Frame:
    return Frame(i, i, np.zeros((3, 4), np.uint8), np.zeros((3, 4), np.float32), INTR)


def test_defaults_match_the_spec():
    cfg = RealSenseConfig()
    assert (cfg.width, cfg.height, cfg.fps, cfg.emitter_enabled) == (848, 480, 30, False)


def test_frame_from_arrays_scales_depth_and_copies_buffers():
    ir = np.full((3, 4), 7, np.uint8)
    units = np.full((3, 4), 1500, np.uint16)
    f = frame_from_arrays(5, 99, ir, units, INTR)
    assert f.index == 5 and f.timestamp_ns == 99
    assert f.depth_m.dtype == np.float32 and np.allclose(f.depth_m, 1.5)
    ir[0, 0] = 0
    assert f.ir[0, 0] == 7  # independent of the SDK buffer


def test_start_retries_then_succeeds():
    attempts, sleeps = [], []

    def start():
        attempts.append(1)
        if len(attempts) < 3:
            raise RuntimeError("failed to set power state")
        return "profile"

    assert start_with_retry(start, attempts=4, delay_s=2.0, sleep=sleeps.append) == "profile"
    assert len(attempts) == 3 and sleeps == [2.0, 2.0]


def test_start_gives_up_with_a_clear_error():
    def start():
        raise RuntimeError("failed to set power state")

    with pytest.raises(FrameSourceError, match="after 4 attempts"):
        start_with_retry(start, attempts=4, delay_s=0.0, sleep=lambda s: None)


def test_queue_keeps_the_newest_frames():
    q = LatestFrameQueue(maxsize=2)
    for i in range(5):
        q.put(frame(i))
    assert q.dropped == 3
    first, second = q.get(0.01), q.get(0.01)
    assert first is not None and second is not None
    assert [first.index, second.index] == [3, 4]
    assert q.get(0.01) is None
