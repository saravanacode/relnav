import numpy as np
import rerun as rr

from relnav.adapters.rerun.sink import RerunSink, height_colors
from relnav.domain.se3 import SE3
from relnav.domain.types import (
    Frame,
    Intrinsics,
    KeyframeView,
    PinReadout,
    TrackingState,
    TrackingStatus,
    WorldState,
)

INTR = Intrinsics(64, 48, 50.0, 50.0, 32.0, 24.0)


def state(index: int, n_keyframes: int, map_version: int) -> WorldState:
    frame = Frame(
        index, index * 1000, np.zeros((48, 64), np.uint8), np.zeros((48, 64), np.float32), INTR
    )
    cloud = np.random.default_rng(index).uniform(-1, 1, (50, 3)).astype(np.float32)
    keyframes = tuple(
        KeyframeView(i, SE3.from_rt(np.eye(3), [0.1 * i, 0, 0]), cloud) for i in range(n_keyframes)
    )
    return WorldState(
        frame=frame,
        status=TrackingStatus(TrackingState.OK, inliers=200),
        T_world_cam=SE3.identity(),
        keyframes=keyframes,
        map_version=map_version,
        path=np.zeros((index + 1, 3)),
        pins=(PinReadout("DOOR", (0.0, 0.0, 3.0), 3.0, 0.0, 0.0),) if index else (),
        events=(f"event {index}",),
    )


def test_height_colors_run_blue_to_red():
    colors = height_colors(np.array([-1.0, 0.75, 3.0]))
    assert colors.shape == (3, 3) and colors.dtype == np.uint8
    assert colors[0, 2] > colors[0, 0]  # low = blue
    assert colors[2, 0] > colors[2, 2]  # high = red


def test_sink_logs_a_run_to_a_recording():
    recording = rr.RecordingStream("relnav-test")
    memory = recording.memory_recording()
    sink = RerunSink(recording)
    sink.publish(state(0, n_keyframes=1, map_version=1))
    sink.publish(state(1, n_keyframes=2, map_version=2))
    sink.publish(state(2, n_keyframes=2, map_version=2))
    sink.close()
    assert sink.logged_clouds == {0, 1}
    assert len(memory.drain_as_bytes()) > 0
