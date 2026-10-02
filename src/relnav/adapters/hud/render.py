"""Draw the HUD image: infrared view with crosshair, plus a status panel on the right."""

from __future__ import annotations

from collections import deque

import cv2
import numpy as np
from numpy.typing import NDArray

from relnav.adapters.presentation import fps, pin_lines, status_lines
from relnav.domain.types import TrackingState, WorldState

PANEL_WIDTH = 380
BADGE_TOP, BADGE_BOTTOM = 44, 78
STATUS_COLORS: dict[TrackingState, tuple[int, int, int]] = {  # BGR
    TrackingState.OK: (80, 200, 80),
    TrackingState.DEGRADED: (0, 165, 255),
    TrackingState.LOST: (60, 60, 230),
    TrackingState.INITIALIZING: (160, 160, 160),
}
_WHITE, _GRAY, _YELLOW = (240, 240, 240), (170, 170, 170), (0, 220, 255)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(image: NDArray[np.uint8], text: str, y: int, color=_WHITE, scale: float = 0.5) -> int:
    cv2.putText(image, text, (12, y), _FONT, scale, color, 1, cv2.LINE_AA)
    return y + int(28 * scale + 8)


class HudRenderer:
    def __init__(self, max_events: int = 4) -> None:
        self._events: deque[str] = deque(maxlen=max_events)

    def render(self, state: WorldState) -> NDArray[np.uint8]:
        self._events.extend(state.events)
        view = cv2.cvtColor(state.frame.ir, cv2.COLOR_GRAY2BGR)
        h, w = view.shape[:2]
        cv2.drawMarker(view, (w // 2, h // 2), _YELLOW, cv2.MARKER_CROSS, 28, 2)

        panel = np.full((h, PANEL_WIDTH, 3), 25, np.uint8)
        _text(panel, "relnav", 30, scale=0.8)
        color = STATUS_COLORS[state.status.state]
        cv2.rectangle(panel, (0, BADGE_TOP), (PANEL_WIDTH - 1, BADGE_BOTTOM), color, -1)
        lines = status_lines(state)
        cv2.putText(panel, lines[0], (12, BADGE_TOP + 24), _FONT, 0.6, (20, 20, 20), 2, cv2.LINE_AA)
        y = BADGE_BOTTOM + 26
        for line in lines[1:]:
            y = _text(panel, line, y)
        y = _text(panel, "PINS", y + 6, _GRAY)
        for line in pin_lines(state) or ["press p to drop a pin"]:
            y = _text(panel, line, y)
        y = _text(panel, "EVENTS", y + 6, _GRAY)
        for event in self._events:
            y = _text(panel, event[:44], y, scale=0.45)
        rate = fps(state)
        footer = f"{rate:4.0f} fps   p = pin   q = quit" if rate else "p = pin   q = quit"
        _text(panel, footer, h - 14, _GRAY, scale=0.45)
        return np.hstack([view, panel]).astype(np.uint8, copy=False)
