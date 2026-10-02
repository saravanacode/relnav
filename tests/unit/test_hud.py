import numpy as np
import pytest

from relnav.adapters.hud.render import (
    BADGE_TOP,
    PANEL_WIDTH,
    STATUS_COLORS,
    HudRenderer,
)
from relnav.adapters.hud.window import HudWindow
from relnav.adapters.presentation import (
    format_bearing,
    fps,
    human_position,
    pin_lines,
    status_lines,
    yaw_deg,
)
from relnav.domain.commands import DropPin, Quit
from relnav.domain.se3 import SE3
from relnav.domain.types import (
    Frame,
    Intrinsics,
    PinPairReadout,
    PinReadout,
    TrackingState,
    TrackingStatus,
    WorldState,
)
from tests.support.synthetic import pose_xz_yaw

INTR = Intrinsics(848, 480, 420.0, 420.0, 424.0, 240.0)


def state(kind: TrackingState = TrackingState.OK) -> WorldState:
    frame = Frame(0, 0, np.full((480, 848), 90, np.uint8), np.zeros((480, 848), np.float32), INTR)
    return WorldState(
        frame=frame,
        status=TrackingStatus(kind, inliers=312),
        T_world_cam=SE3.from_rt(np.eye(3), [1.24, -0.02, 2.10]),
        pins=(PinReadout("DOOR", (0.0, 0.0, 3.0), 2.31, 35.2, 0.0),),
        pin_pairs=(PinPairReadout("DOOR", "CHAIR", 2.41),),
        events=("pin DOOR dropped",),
        timings_ms={"total_ms": 20.0},
    )


def test_human_position_points_y_up():
    assert human_position(SE3.from_rt(np.eye(3), [1.0, -2.0, 3.0])) == (1.0, 2.0, 3.0)


def test_yaw_is_positive_to_the_right():
    assert yaw_deg(pose_xz_yaw(0.0, 0.0, 42.0)) == pytest.approx(42.0)


def test_bearing_text():
    assert format_bearing(35.2) == "35 deg R"
    assert format_bearing(-12.0) == "12 deg L"


def test_text_lines():
    s = state()
    assert status_lines(s)[0] == "STATUS  OK  (312 inliers)"
    assert "POS  X +1.24  Y +0.02  Z +2.10 m" in status_lines(s)
    assert pin_lines(s) == ["DOOR      2.31 m  35 deg R", "DOOR-CHAIR  2.41 m"]
    assert fps(s) == pytest.approx(50.0)


def test_render_size_and_status_badge():
    renderer = HudRenderer()
    image = renderer.render(state(TrackingState.OK))
    assert image.shape == (480, 848 + PANEL_WIDTH, 3) and image.dtype == np.uint8
    # Sample the badge's top-right corner, away from the status text.
    badge_pixel = image[BADGE_TOP + 3, 848 + PANEL_WIDTH - 6]
    assert tuple(int(c) for c in badge_pixel) == STATUS_COLORS[TrackingState.OK]
    lost = renderer.render(state(TrackingState.LOST))
    lost_pixel = lost[BADGE_TOP + 3, 848 + PANEL_WIDTH - 6]
    assert tuple(int(c) for c in lost_pixel) == STATUS_COLORS[TrackingState.LOST]


def test_window_keys_become_commands():
    window = HudWindow.__new__(HudWindow)  # no OpenCV window needed for key mapping
    window._keys = []
    for key in (ord("p"), ord("x"), ord("q")):
        window.handle_key(key)
    assert window.poll() == [DropPin(), Quit()]
    assert window.poll() == []
