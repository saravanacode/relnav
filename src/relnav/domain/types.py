"""Plain data types shared by every layer."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from numpy.typing import NDArray

from relnav.domain.se3 import SE3


class TrackingState(StrEnum):
    INITIALIZING = "INITIALIZING"
    OK = "OK"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole model of the depth / left-IR camera (they share one pixel grid)."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float = 0.001  # meters per raw depth unit

    @property
    def K(self) -> NDArray[np.float64]:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass(frozen=True, eq=False)
class Frame:
    index: int
    timestamp_ns: int
    ir: NDArray[np.uint8]  # (H, W) left infrared image
    depth_m: NDArray[np.float32]  # (H, W) meters, 0 = no depth
    intrinsics: Intrinsics


@dataclass(eq=False)
class Keyframe:
    """A frame kept in the map. The backend rewrites T_world_cam after loop closure."""

    id: int
    frame_index: int
    T_world_cam: SE3
    keypoints: NDArray[np.float32]  # (N, 2) pixels
    descriptors: NDArray[np.uint8]  # (N, 32) ORB
    points_cam: NDArray[np.float64]  # (N, 3) feature points in this keyframe's camera frame
    cloud_cam: NDArray[np.float32]  # (M, 3) sparse display points, camera frame


@dataclass(frozen=True, eq=False)
class Pin:
    """A user-marked 3D point, stored relative to a keyframe so it follows map corrections."""

    id: int
    label: str
    anchor_keyframe_id: int
    point_in_keyframe: NDArray[np.float64]  # (3,)


@dataclass(frozen=True)
class TrackingStatus:
    state: TrackingState
    inliers: int = 0
    reprojection_rmse_px: float = 0.0
    message: str = ""


@dataclass(frozen=True)
class PinReadout:
    label: str
    position_world: tuple[float, float, float]
    distance_m: float  # from the camera
    azimuth_deg: float  # positive = right of the camera's forward axis
    elevation_deg: float  # positive = above


@dataclass(frozen=True)
class PinPairReadout:
    label_a: str
    label_b: str
    distance_m: float


@dataclass(frozen=True, eq=False)
class KeyframeView:
    id: int
    T_world_cam: SE3
    cloud_cam: NDArray[np.float32]


def _empty_path() -> NDArray[np.float64]:
    return np.zeros((0, 3))


@dataclass(frozen=True, eq=False)
class WorldState:
    """Snapshot published to sinks after every frame."""

    frame: Frame
    status: TrackingStatus
    T_world_cam: SE3 | None = None
    keyframes: tuple[KeyframeView, ...] = ()
    map_version: int = 0  # changes when a keyframe is added or poses are corrected
    path: NDArray[np.float64] = field(default_factory=_empty_path)
    pins: tuple[PinReadout, ...] = ()
    pin_pairs: tuple[PinPairReadout, ...] = ()
    events: tuple[str, ...] = ()
    timings_ms: Mapping[str, float] = field(default_factory=dict)

    @classmethod
    def preview(cls, frame: Frame, message: str, events: tuple[str, ...] = ()) -> WorldState:
        """State for runs without SLAM (recording): camera feed and a message only."""
        status = TrackingStatus(TrackingState.INITIALIZING, message=message)
        return cls(frame=frame, status=status, events=events)
