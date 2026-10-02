"""Tracking health: OK / DEGRADED / LOST, in the spirit of flight-software fault detection."""

from __future__ import annotations

from dataclasses import dataclass

from relnav.core.tracker import TrackResult
from relnav.domain.types import TrackingState, TrackingStatus


@dataclass(frozen=True)
class HealthConfig:
    degraded_min_inliers: int = 60
    degraded_max_rmse_px: float = 2.0
    lost_after_failures: int = 3
    init_min_features: int = 100
    relocalization_min_inliers: int = 60


class HealthMonitor:
    def __init__(self, cfg: HealthConfig) -> None:
        self.cfg = cfg
        self._state = TrackingState.INITIALIZING
        self._failures = 0

    @property
    def state(self) -> TrackingState:
        return self._state

    def initialized(self) -> TrackingStatus:
        self._state = TrackingState.OK
        self._failures = 0
        return TrackingStatus(TrackingState.OK, message="initialized")

    def update(self, result: TrackResult) -> TrackingStatus:
        if result.ok:
            self._failures = 0
            self._state = self._quality(result)
            return TrackingStatus(self._state, result.inliers, result.rmse_px)
        self._failures += 1
        lost = self._failures >= self.cfg.lost_after_failures
        self._state = TrackingState.LOST if lost else TrackingState.DEGRADED
        return TrackingStatus(self._state, result.inliers, result.rmse_px, "tracking failed")

    def relocalized(self, result: TrackResult) -> TrackingStatus:
        self._failures = 0
        self._state = self._quality(result)
        return TrackingStatus(self._state, result.inliers, result.rmse_px, "relocalized")

    def still_lost(self) -> TrackingStatus:
        return TrackingStatus(TrackingState.LOST, message="searching keyframes")

    def _quality(self, result: TrackResult) -> TrackingState:
        weak = (
            result.inliers < self.cfg.degraded_min_inliers
            or result.rmse_px > self.cfg.degraded_max_rmse_px
        )
        return TrackingState.DEGRADED if weak else TrackingState.OK
