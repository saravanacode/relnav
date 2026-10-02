"""Needs the D435i and root. Run:

    sudo env RELNAV_HARDWARE=1 PYTHONDONTWRITEBYTECODE=1 \
        "$(command -v uv)" run --no-sync pytest -m hardware
"""

import os
from itertools import pairwise

import numpy as np
import pytest

pytestmark = [
    pytest.mark.hardware,
    pytest.mark.skipif(
        os.environ.get("RELNAV_HARDWARE") != "1" or os.geteuid() != 0,
        reason="needs the camera and root (set RELNAV_HARDWARE=1 and run with sudo)",
    ),
]


def test_camera_streams_depth_and_infrared():
    from relnav.adapters.realsense.source import RealSenseFrameSource

    source = RealSenseFrameSource()
    frames = []
    try:
        for _ in range(30):
            frame = source.next_frame(2.0)
            assert frame is not None
            frames.append(frame)
    finally:
        source.close()
    last = frames[-1]
    assert last.ir.shape == (480, 848) and last.depth_m.shape == (480, 848)
    assert np.count_nonzero(last.depth_m) > 0.2 * last.depth_m.size
    assert all(b.timestamp_ns > a.timestamp_ns for a, b in pairwise(frames))
