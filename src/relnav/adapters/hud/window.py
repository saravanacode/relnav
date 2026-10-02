"""OpenCV window that shows the HUD and turns key presses into commands."""

from __future__ import annotations

import cv2

from relnav.adapters.hud.render import HudRenderer
from relnav.domain.commands import Command, DropPin, Quit
from relnav.domain.types import WorldState

_ESC = 27


class HudWindow:
    """StateSink and CommandSource. Keys read while publishing apply to the next frame."""

    def __init__(self, title: str = "relnav") -> None:
        self._title = title
        self._renderer = HudRenderer()
        self._keys: list[int] = []
        cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)

    def publish(self, state: WorldState) -> None:
        cv2.imshow(self._title, self._renderer.render(state))
        key = cv2.waitKey(1) & 0xFF
        if key != 0xFF:
            self.handle_key(key)

    def handle_key(self, key: int) -> None:
        self._keys.append(key)

    def poll(self) -> list[Command]:
        commands: list[Command] = []
        for key in self._keys:
            if key == ord("p"):
                commands.append(DropPin())
            elif key in (ord("q"), _ESC):
                commands.append(Quit())
        self._keys.clear()
        return commands

    def close(self) -> None:
        try:
            cv2.destroyWindow(self._title)
            cv2.waitKey(1)
        except cv2.error:
            pass
