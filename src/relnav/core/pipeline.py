"""SlamPipeline: one call per frame; ties tracking, mapping, pins and health together.

No I/O and no wall clock in decisions, so the same frames and commands always give the
same trajectory and pins (REQ-REP-01). Timings are measured for reporting only.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np

from relnav.core.backend import Backend, BackendConfig
from relnav.core.features import FeatureConfig, FeatureExtractor, Features
from relnav.core.health import HealthConfig, HealthMonitor
from relnav.core.keyframes import KeyframePolicy, KeyframeStore
from relnav.core.pins import PinConfig, PinRegistry
from relnav.core.tracker import MotionModel, Tracker, TrackerConfig
from relnav.domain.commands import Command, DropPin
from relnav.domain.geometry import sparse_cloud
from relnav.domain.se3 import SE3
from relnav.domain.types import (
    Frame,
    Keyframe,
    KeyframeView,
    TrackingState,
    TrackingStatus,
    WorldState,
)

_TRACKING = (TrackingState.OK, TrackingState.DEGRADED)


@dataclass(frozen=True)
class SlamConfig:
    features: FeatureConfig = field(default_factory=FeatureConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    keyframes: KeyframePolicy = field(default_factory=KeyframePolicy)
    backend: BackendConfig = field(default_factory=BackendConfig)
    health: HealthConfig = field(default_factory=HealthConfig)
    pins: PinConfig = field(default_factory=PinConfig)
    cloud_stride: int = 8
    relocalization_nearest: int = 10


def _ms_since(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


class SlamPipeline:
    def __init__(self, cfg: SlamConfig | None = None) -> None:
        self.cfg = cfg or SlamConfig()
        cv2.setRNGSeed(0)
        self._extractor = FeatureExtractor(self.cfg.features)
        self._tracker = Tracker(self.cfg.tracker, self.cfg.features.ratio)
        self._keyframes = KeyframeStore()
        self._backend = Backend(self.cfg.backend, self.cfg.tracker, self.cfg.features.ratio)
        self._health = HealthMonitor(self.cfg.health)
        self._pins = PinRegistry(self.cfg.pins)
        self._motion = MotionModel()
        self._T_world_cam: SE3 | None = None
        self._reference: Keyframe | None = None
        self._status = TrackingStatus(TrackingState.INITIALIZING, message="waiting for texture")
        self._trajectory: list[tuple[int, int, SE3]] = []  # (timestamp_ns, keyframe id, T_kf_cam)
        self._path: list[np.ndarray] = []
        self._map_version = 0
        self._loops = 0
        self._tracked_now = False  # True only when this frame produced its own pose

    @property
    def keyframes(self) -> KeyframeStore:
        return self._keyframes

    @property
    def pins(self) -> PinRegistry:
        return self._pins

    @property
    def loop_count(self) -> int:
        return self._loops

    def trajectory(self) -> list[tuple[int, SE3]]:
        return [
            (timestamp, self._keyframes[kf_id].T_world_cam @ T_kf_cam)
            for timestamp, kf_id, T_kf_cam in self._trajectory
        ]

    def step(self, frame: Frame, commands: Sequence[Command] = ()) -> WorldState:
        start = time.perf_counter()
        events: list[str] = []
        timings: dict[str, float] = {}
        features = self._extractor.extract(frame)
        timings["features_ms"] = _ms_since(start)

        tracking_start = time.perf_counter()
        self._tracked_now = False
        state = self._health.state
        if state is TrackingState.INITIALIZING:
            self._initialize(frame, features, events)
        elif state is TrackingState.LOST:
            self._relocalize(frame, features, events)
        else:
            self._track(frame, features, events)
        timings["tracking_ms"] = _ms_since(tracking_start)

        for command in commands:
            if isinstance(command, DropPin):
                self._drop_pin(frame, command, events)

        if self._tracked_now and self._T_world_cam is not None:
            assert self._reference is not None
            T_kf_cam = self._reference.T_world_cam.inverse() @ self._T_world_cam
            self._trajectory.append((frame.timestamp_ns, self._reference.id, T_kf_cam))
            self._path.append(self._T_world_cam.t.copy())
        timings["total_ms"] = _ms_since(start)
        return self._snapshot(frame, events, timings)

    def _initialize(self, frame: Frame, features: Features, events: list[str]) -> None:
        if len(features) < self.cfg.health.init_min_features:
            message = f"need texture: {len(features)} features"
            self._status = TrackingStatus(TrackingState.INITIALIZING, message=message)
            return
        self._T_world_cam = SE3.identity()
        self._insert_keyframe(frame, features, len(features), events)
        self._motion.update(self._T_world_cam)
        self._status = self._health.initialized()
        self._tracked_now = True
        events.append("initialized: world origin set")

    def _track(self, frame: Frame, features: Features, events: list[str]) -> None:
        assert self._reference is not None
        guess = self._motion.predict()
        result = self._tracker.track(features, self._reference, frame.intrinsics, guess)
        self._status = self._health.update(result)
        if not result.ok or result.T_world_cam is None:
            if self._status.state is TrackingState.LOST:
                self._motion.reset()
                events.append("tracking lost")
            return
        self._T_world_cam = result.T_world_cam
        self._motion.update(result.T_world_cam)
        self._tracked_now = True
        enough_texture = len(features) >= self.cfg.health.init_min_features
        if enough_texture and self.cfg.keyframes.needs_keyframe(
            self._reference, result.T_world_cam, result.inliers
        ):
            self._insert_keyframe(frame, features, result.inliers, events)

    def _relocalize(self, frame: Frame, features: Features, events: list[str]) -> None:
        nearest = (
            self._keyframes.nearest(self._T_world_cam, k=self.cfg.relocalization_nearest)
            if self._T_world_cam is not None
            else []
        )
        min_inliers = self.cfg.health.relocalization_min_inliers
        found = self._tracker.relocalize(features, nearest, frame.intrinsics, min_inliers)
        if found is None:
            tried = {kf.id for kf in nearest}
            rest = [kf for kf in self._keyframes if kf.id not in tried]
            found = self._tracker.relocalize(features, rest, frame.intrinsics, min_inliers)
        if found is None or found[0].T_world_cam is None:
            self._status = self._health.still_lost()
            return
        result, keyframe = found
        T_world_cam = result.T_world_cam
        assert T_world_cam is not None
        self._T_world_cam = T_world_cam
        self._reference = keyframe
        self._motion.reset()
        self._motion.update(T_world_cam)
        self._status = self._health.relocalized(result)
        self._tracked_now = True
        events.append(f"relocalized against keyframe {keyframe.id}")

    def _insert_keyframe(
        self, frame: Frame, features: Features, inliers: int, events: list[str]
    ) -> None:
        assert self._T_world_cam is not None
        cloud = sparse_cloud(frame.depth_m, frame.intrinsics, self.cfg.cloud_stride)
        keyframe = self._keyframes.add(frame.index, self._T_world_cam, features, cloud)
        loop = self._backend.add_keyframe(keyframe, self._keyframes, inliers, frame.intrinsics)
        self._reference = keyframe
        self._map_version += 1
        if loop is None:
            return
        self._loops += 1
        self._T_world_cam = keyframe.T_world_cam  # the backend corrected it
        self._motion.reset()
        self._motion.update(keyframe.T_world_cam)
        self._path = [T.t.copy() for _, T in self.trajectory()]
        events.append(
            f"loop closed: keyframe {loop.new_id} <-> {loop.old_id} ({loop.inliers} inliers)"
        )

    def _drop_pin(self, frame: Frame, command: DropPin, events: list[str]) -> None:
        if (
            self._status.state not in _TRACKING
            or self._T_world_cam is None
            or self._reference is None
        ):
            label = self._pins.claim_label(command.label)
            events.append(f"pin {label} rejected: tracking {self._status.state}")
            return
        if not self._tracked_now:
            label = self._pins.claim_label(command.label)
            events.append(f"pin {label} rejected: tracking failed on this frame")
            return
        u = command.u if command.u is not None else frame.intrinsics.width // 2
        v = command.v if command.v is not None else frame.intrinsics.height // 2
        result = self._pins.drop(frame, u, v, self._T_world_cam, self._reference, command.label)
        if result.pin is None:
            events.append(f"pin {result.label} rejected: {result.reason}")
        else:
            events.append(f"pin {result.pin.label} dropped")

    def _snapshot(self, frame: Frame, events: list[str], timings: dict[str, float]) -> WorldState:
        T = self._T_world_cam
        return WorldState(
            frame=frame,
            status=self._status,
            T_world_cam=T,
            keyframes=tuple(
                KeyframeView(kf.id, kf.T_world_cam, kf.cloud_cam) for kf in self._keyframes
            ),
            map_version=self._map_version,
            path=np.array(self._path, dtype=np.float64).reshape(-1, 3),
            pins=self._pins.readouts(T, self._keyframes) if T is not None else (),
            pin_pairs=self._pins.pairs(self._keyframes),
            events=tuple(events),
            timings_ms=timings,
        )
