import dataclasses

import numpy as np
import pytest

from relnav.core.pins import PinConfig
from relnav.core.pipeline import SlamConfig, SlamPipeline
from relnav.domain.commands import DropPin
from relnav.domain.se3 import SE3
from relnav.domain.types import TrackingState
from tests.support.synthetic import (
    FRAME_PERIOD_NS,
    SMALL_INTRINSICS,
    SyntheticRoom,
    blank_frame,
    loop_trajectory,
)

pytestmark = pytest.mark.slow
TRACKING = (TrackingState.OK, TrackingState.DEGRADED)


@pytest.fixture(scope="module")
def room():
    return SyntheticRoom()


@pytest.fixture(scope="module")
def loop_run(room):
    poses = loop_trajectory(150)
    pipeline = SlamPipeline()
    states = [pipeline.step(room.render(T, index=i)) for i, T in enumerate(poses)]
    return poses, pipeline, states


def test_tracks_the_whole_loop(loop_run):
    _, _, states = loop_run
    tracked = sum(s.status.state in TRACKING for s in states)
    assert tracked >= 0.95 * len(states)
    assert states[0].events == ("initialized: world origin set",)


def test_loop_is_closed(loop_run):
    _, pipeline, states = loop_run
    assert pipeline.loop_count >= 1
    assert any(e.startswith("loop closed") for s in states for e in s.events)


def test_trajectory_error_under_2_cm(loop_run):
    """REQ-ACC-03: absolute trajectory error (RMSE) < 2 cm on the synthetic loop."""
    poses, pipeline, _ = loop_run
    estimated = dict(pipeline.trajectory())
    errors = [
        np.linalg.norm(estimated[i * FRAME_PERIOD_NS].t - T.t)
        for i, T in enumerate(poses)
        if i * FRAME_PERIOD_NS in estimated
    ]
    assert len(errors) >= 0.95 * len(poses)
    assert float(np.sqrt(np.mean(np.square(errors)))) < 0.02


def test_world_state_carries_map_and_timings(loop_run):
    _, pipeline, states = loop_run
    last = states[-1]
    assert len(last.keyframes) == len(pipeline.keyframes) >= 10
    assert last.path.shape[1] == 3 and len(last.path) >= 0.95 * len(states)
    assert {"features_ms", "tracking_ms", "total_ms"} <= set(last.timings_ms)
    assert last.map_version >= len(last.keyframes)


def test_lost_then_recovers(room):
    """REQ-REL-02: covered lens -> LOST; camera back on a mapped area -> OK again."""
    poses = loop_trajectory(150)[:60]
    frames = [
        blank_frame(i) if 30 <= i < 36 else room.render(T, index=i) for i, T in enumerate(poses)
    ]
    pipeline = SlamPipeline()
    states = [pipeline.step(f) for f in frames]
    st = [s.status.state for s in states]
    assert TrackingState.LOST in st[30:36]
    assert all(s in TRACKING for s in st[38:])
    events = [e for s in states for e in s.events]
    assert "tracking lost" in events
    assert any(e.startswith("relocalized against keyframe") for e in events)


def test_pin_on_the_wall(room):
    frame = room.render(SE3.identity(), index=0, depth_noise=False)
    pipeline = SlamPipeline(SlamConfig(pins=PinConfig(labels=("WALL",))))
    pipeline.step(frame)
    again = dataclasses.replace(frame, index=1, timestamp_ns=FRAME_PERIOD_NS)
    state = pipeline.step(again, [DropPin()])
    assert "pin WALL dropped" in state.events
    assert state.pins[0].position_world == pytest.approx((0.0, 0.0, 3.0), abs=0.01)
    assert state.pins[0].distance_m == pytest.approx(3.0, abs=0.01)


def test_pin_rejected_before_initialization():
    """Review focus 2: a pin while INITIALIZING is rejected, not a crash."""
    state = SlamPipeline().step(blank_frame(0), [DropPin()])
    assert state.events == ("pin P1 rejected: tracking INITIALIZING",)
    assert state.pins == ()


def test_runs_at_other_resolution(room):
    """Review focus 5: nothing hard-codes 848x480."""
    poses = loop_trajectory(150)[:30]
    pipeline = SlamPipeline()
    states = [
        pipeline.step(room.render(T, index=i, intrinsics=SMALL_INTRINSICS))
        for i, T in enumerate(poses)
    ]
    assert all(s.status.state in TRACKING for s in states)


def test_same_input_gives_same_output(room):
    """REQ-REP-01 at the core level."""
    frames = [room.render(T, index=i) for i, T in enumerate(loop_trajectory(150)[:40])]
    runs = []
    for _ in range(2):
        pipeline = SlamPipeline()
        for frame in frames:
            pipeline.step(frame)
        runs.append(pipeline.trajectory())
    assert len(runs[0]) == len(runs[1])
    for (ta, Ta), (tb, Tb) in zip(runs[0], runs[1], strict=True):
        assert ta == tb and np.array_equal(Ta.matrix, Tb.matrix)


def test_pin_rejected_for_tracking_still_uses_up_its_label(room):
    pipeline = SlamPipeline(SlamConfig(pins=PinConfig(labels=("DOOR", "CHAIR"))))
    early = pipeline.step(blank_frame(0), [DropPin()])
    assert early.events == ("pin DOOR rejected: tracking INITIALIZING",)
    frame = room.render(SE3.identity(), index=1, depth_noise=False)
    pipeline.step(frame)
    again = dataclasses.replace(frame, index=2, timestamp_ns=2 * FRAME_PERIOD_NS)
    assert "pin CHAIR dropped" in pipeline.step(again, [DropPin()]).events
