"""Camera pose from 3D-2D matches (PnP + RANSAC), constant-velocity prediction, relocalization."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from relnav.core.features import Features, match
from relnav.domain.geometry import project
from relnav.domain.se3 import SE3
from relnav.domain.types import Intrinsics, Keyframe


@dataclass(frozen=True)
class TrackerConfig:
    ransac_reprojection_px: float = 3.0
    ransac_iterations: int = 100
    min_inliers: int = 20


@dataclass(frozen=True, eq=False)
class TrackResult:
    ok: bool
    T_world_cam: SE3 | None
    inliers: int
    rmse_px: float

    @staticmethod
    def failed(inliers: int = 0) -> TrackResult:
        return TrackResult(False, None, inliers, float("inf"))


def _ransac(
    obj: NDArray[np.float64],
    img: NDArray[np.float64],
    K: NDArray[np.float64],
    cfg: TrackerConfig,
    guess: SE3 | None,
) -> tuple[bool, Any, Any, Any]:
    """OpenCV PnP RANSAC; returns (ok, rvec, tvec, inlier indices or None)."""
    if guess is not None:
        rvec, tvec = guess.inverse().rvec_tvec()  # OpenCV wants T_cam_world
        return cv2.solvePnPRansac(
            obj,
            img,
            K,
            None,
            rvec=rvec.reshape(3, 1).copy(),
            tvec=tvec.reshape(3, 1).copy(),
            useExtrinsicGuess=True,
            iterationsCount=cfg.ransac_iterations,
            reprojectionError=cfg.ransac_reprojection_px,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    return cv2.solvePnPRansac(
        obj,
        img,
        K,
        None,
        iterationsCount=cfg.ransac_iterations,
        reprojectionError=cfg.ransac_reprojection_px,
        flags=cv2.SOLVEPNP_SQPNP,
    )


def estimate_pose(
    points_world: NDArray[np.float64],
    keypoints: NDArray[np.floating],
    intr: Intrinsics,
    cfg: TrackerConfig,
    guess: SE3 | None = None,
) -> TrackResult:
    """Camera pose in the frame of ``points_world`` from matched pixels ``keypoints``.

    A guess seeds the solver. OpenCV's guided RANSAC can report success with a wrong pose
    when the guess is bad, so a guided result is kept only if its refined reprojection
    error is within the RANSAC threshold; otherwise the unguided SQPnP solve is used.
    """
    if len(points_world) < 6:
        return TrackResult.failed()
    obj = np.ascontiguousarray(points_world, dtype=np.float64)
    img = np.ascontiguousarray(keypoints, dtype=np.float64)
    if guess is not None:
        guided = _solve(obj, img, intr, cfg, guess)
        if guided.ok and guided.rmse_px <= cfg.ransac_reprojection_px:
            return guided
    return _solve(obj, img, intr, cfg, None)


def _solve(
    obj: NDArray[np.float64],
    img: NDArray[np.float64],
    intr: Intrinsics,
    cfg: TrackerConfig,
    guess: SE3 | None,
) -> TrackResult:
    ok, rvec, tvec, inliers = _ransac(obj, img, intr.K, cfg, guess)
    if not ok or inliers is None or len(inliers) < cfg.min_inliers:
        return TrackResult.failed(0 if inliers is None else len(inliers))
    idx = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(obj[idx], img[idx], intr.K, None, rvec, tvec)
    T_cam_world = SE3.from_rvec_tvec(rvec.ravel(), tvec.ravel())
    residuals = project(T_cam_world.apply(obj[idx]), intr) - img[idx]
    rmse = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))
    return TrackResult(True, T_cam_world.inverse(), len(idx), rmse)


class MotionModel:
    """Constant velocity: the next pose repeats the last frame-to-frame motion."""

    def __init__(self) -> None:
        self._previous: SE3 | None = None
        self._last: SE3 | None = None

    def update(self, T_world_cam: SE3) -> None:
        self._previous, self._last = self._last, T_world_cam

    def predict(self) -> SE3 | None:
        if self._last is None:
            return None
        if self._previous is None:
            return self._last
        return self._last @ (self._previous.inverse() @ self._last)

    def reset(self) -> None:
        self._previous = self._last = None


class Tracker:
    def __init__(self, cfg: TrackerConfig, ratio: float) -> None:
        self.cfg = cfg
        self.ratio = ratio

    def track(
        self, features: Features, reference: Keyframe, intr: Intrinsics, guess: SE3 | None
    ) -> TrackResult:
        """Pose of the current frame from matches against a keyframe with known 3D points."""
        m = match(features.descriptors, reference.descriptors, self.ratio)
        if len(m) < 6:
            return TrackResult.failed()
        points_world = reference.T_world_cam.apply(reference.points_cam[m[:, 1]])
        return estimate_pose(points_world, features.keypoints[m[:, 0]], intr, self.cfg, guess)

    def relocalize(
        self,
        features: Features,
        candidates: Sequence[Keyframe],
        intr: Intrinsics,
        min_inliers: int,
    ) -> tuple[TrackResult, Keyframe] | None:
        """Try each candidate keyframe; keep the one with the most inliers."""
        best: tuple[TrackResult, Keyframe] | None = None
        for keyframe in candidates:
            result = self.track(features, keyframe, intr, None)
            better = best is None or result.inliers > best[0].inliers
            if result.ok and result.inliers >= min_inliers and better:
                best = (result, keyframe)
        return best
