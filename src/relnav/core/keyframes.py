"""When to add a keyframe, and the store that holds them."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from relnav.core.features import Features
from relnav.domain.se3 import SE3
from relnav.domain.types import Keyframe


@dataclass(frozen=True)
class KeyframePolicy:
    max_translation_m: float = 0.15
    max_rotation_deg: float = 15.0
    min_inlier_ratio: float = 0.5
    min_inliers: int = 100

    def needs_keyframe(self, reference: Keyframe, T_world_cam: SE3, inliers: int) -> bool:
        motion = reference.T_world_cam.inverse() @ T_world_cam
        return (
            motion.translation_norm() > self.max_translation_m
            or motion.rotation_angle_deg() > self.max_rotation_deg
            or inliers < self.min_inlier_ratio * len(reference.keypoints)
            or inliers < self.min_inliers
        )


def viewing_angle_deg(a: SE3, b: SE3) -> float:
    """Angle between the two cameras' forward (z) axes."""
    cos = float(np.clip(a.R[:, 2] @ b.R[:, 2], -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


class KeyframeStore:
    def __init__(self) -> None:
        self._keyframes: list[Keyframe] = []

    def add(
        self,
        frame_index: int,
        T_world_cam: SE3,
        features: Features,
        cloud_cam: NDArray[np.float32],
    ) -> Keyframe:
        keyframe = Keyframe(
            id=len(self._keyframes),
            frame_index=frame_index,
            T_world_cam=T_world_cam,
            keypoints=features.keypoints,
            descriptors=features.descriptors,
            points_cam=features.points_cam,
            cloud_cam=cloud_cam,
        )
        self._keyframes.append(keyframe)
        return keyframe

    def __len__(self) -> int:
        return len(self._keyframes)

    def __getitem__(self, keyframe_id: int) -> Keyframe:
        return self._keyframes[keyframe_id]

    def __iter__(self) -> Iterator[Keyframe]:
        return iter(self._keyframes)

    @property
    def latest(self) -> Keyframe | None:
        return self._keyframes[-1] if self._keyframes else None

    def nearest(
        self,
        T_world_cam: SE3,
        k: int | None = None,
        max_distance_m: float | None = None,
        max_angle_deg: float | None = None,
        exclude_ids: frozenset[int] = frozenset(),
    ) -> list[Keyframe]:
        scored = []
        for kf in self._keyframes:
            if kf.id in exclude_ids:
                continue
            distance = float(np.linalg.norm(kf.T_world_cam.t - T_world_cam.t))
            if max_distance_m is not None and distance > max_distance_m:
                continue
            if max_angle_deg is not None and (
                viewing_angle_deg(kf.T_world_cam, T_world_cam) > max_angle_deg
            ):
                continue
            scored.append((distance, kf.id, kf))
        scored.sort(key=lambda item: (item[0], item[1]))
        chosen = [kf for _, _, kf in scored]
        return chosen if k is None else chosen[:k]
