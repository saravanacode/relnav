import numpy as np

from relnav.domain.commands import DropPin, Quit
from relnav.domain.types import Frame, Intrinsics, TrackingState, WorldState


def test_intrinsic_matrix():
    intr = Intrinsics(width=848, height=480, fx=420.0, fy=421.0, cx=424.0, cy=240.0)
    assert np.allclose(intr.K, [[420.0, 0, 424.0], [0, 421.0, 240.0], [0, 0, 1]])
    assert intr.depth_scale == 0.001


def test_preview_state_has_no_pose():
    intr = Intrinsics(4, 3, 2.0, 2.0, 2.0, 1.5)
    frame = Frame(0, 0, np.zeros((3, 4), np.uint8), np.zeros((3, 4), np.float32), intr)
    state = WorldState.preview(frame, "recording", events=("pin marked",))
    assert state.T_world_cam is None
    assert state.status.state is TrackingState.INITIALIZING
    assert state.status.message == "recording"
    assert state.events == ("pin marked",)
    assert state.path.shape == (0, 3)


def test_commands_are_values():
    assert DropPin() == DropPin(label=None, u=None, v=None)
    assert Quit() == Quit()
    assert str(TrackingState.LOST) == "LOST"
