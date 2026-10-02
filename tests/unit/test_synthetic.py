from itertools import pairwise

import cv2
import numpy as np
import pytest

from relnav.domain.se3 import SE3
from tests.support.synthetic import (
    FRAME_PERIOD_NS,
    SMALL_INTRINSICS,
    SyntheticRoom,
    blank_frame,
    loop_trajectory,
    pose_xz_yaw,
)


@pytest.fixture(scope="module")
def room():
    return SyntheticRoom()


def test_wall_depth_is_exact_without_noise(room):
    frame = room.render(SE3.identity(), depth_noise=False)
    assert frame.ir.shape == (480, 848) and frame.ir.dtype == np.uint8
    assert frame.depth_m.dtype == np.float32
    assert frame.depth_m[240, 424] == pytest.approx(3.0, abs=1e-4)


def test_texture_gives_orb_plenty_of_corners(room):
    frame = room.render(SE3.identity())
    assert frame.ir.std() > 30
    assert len(cv2.ORB.create(2000).detect(frame.ir, None)) >= 500


def test_index_sets_timestamp(room):
    frame = room.render(SE3.identity(), index=3, intrinsics=SMALL_INTRINSICS)
    assert frame.timestamp_ns == 3 * FRAME_PERIOD_NS
    assert frame.ir.shape == (240, 424)


def test_yaw_turns_right():
    T = pose_xz_yaw(0.0, 0.0, 90.0)
    assert np.allclose(T.R[:, 2], [1.0, 0.0, 0.0])  # camera forward axis now points to +x


def test_loop_returns_near_start():
    poses = loop_trajectory(150)
    assert poses[0].allclose(SE3.identity())
    assert np.linalg.norm(poses[-1].t - poses[0].t) < 0.03
    steps = [np.linalg.norm(b.t - a.t) for a, b in pairwise(poses)]
    assert max(steps) < 0.03


def test_blank_frame_is_empty():
    frame = blank_frame(7)
    assert frame.index == 7
    assert not frame.ir.any() and not frame.depth_m.any()
