"""Pins: user-marked 3D points, anchored to keyframes so they follow map corrections."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import numpy as np
from numpy.typing import NDArray

from relnav.core.keyframes import KeyframeStore
from relnav.domain.geometry import bearing, deproject, depth_patch_median
from relnav.domain.se3 import SE3
from relnav.domain.types import Frame, Keyframe, Pin, PinPairReadout, PinReadout


@dataclass(frozen=True)
class PinConfig:
    labels: tuple[str, ...] = ()
    patch_half: int = 4  # 9x9 patch
    min_valid_fraction: float = 0.3


@dataclass(frozen=True)
class PinDropResult:
    pin: Pin | None
    reason: str = ""


class PinRegistry:
    def __init__(self, cfg: PinConfig) -> None:
        self.cfg = cfg
        self._pins: list[Pin] = []

    @property
    def pins(self) -> tuple[Pin, ...]:
        return tuple(self._pins)

    def drop(
        self,
        frame: Frame,
        u: int,
        v: int,
        T_world_cam: SE3,
        reference: Keyframe,
        label: str | None = None,
    ) -> PinDropResult:
        z = depth_patch_median(
            frame.depth_m, u, v, self.cfg.patch_half, self.cfg.min_valid_fraction
        )
        if z is None:
            return PinDropResult(None, "no depth at crosshair")
        point_cam = deproject([[u, v]], [z], frame.intrinsics)[0]
        point_in_keyframe = (reference.T_world_cam.inverse() @ T_world_cam).apply(point_cam)
        n = len(self._pins)
        if label is None:
            label = self.cfg.labels[n] if n < len(self.cfg.labels) else f"P{n + 1}"
        pin = Pin(
            id=n, label=label, anchor_keyframe_id=reference.id, point_in_keyframe=point_in_keyframe
        )
        self._pins.append(pin)
        return PinDropResult(pin)

    def world_position(self, pin: Pin, keyframes: KeyframeStore) -> NDArray[np.float64]:
        return keyframes[pin.anchor_keyframe_id].T_world_cam.apply(pin.point_in_keyframe)

    def readouts(self, T_world_cam: SE3, keyframes: KeyframeStore) -> tuple[PinReadout, ...]:
        T_cam_world = T_world_cam.inverse()
        out = []
        for pin in self._pins:
            world = self.world_position(pin, keyframes)
            distance, azimuth, elevation = bearing(T_cam_world.apply(world))
            position = (float(world[0]), float(world[1]), float(world[2]))
            out.append(PinReadout(pin.label, position, distance, azimuth, elevation))
        return tuple(out)

    def pairs(self, keyframes: KeyframeStore) -> tuple[PinPairReadout, ...]:
        located = [(pin.label, self.world_position(pin, keyframes)) for pin in self._pins]
        return tuple(
            PinPairReadout(a, b, float(np.linalg.norm(pa - pb)))
            for (a, pa), (b, pb) in combinations(located, 2)
        )
