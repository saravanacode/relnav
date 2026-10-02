"""Pose graph over keyframes: odometry edges, proximity loop closure, global optimization."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d

from relnav.core.features import match
from relnav.core.keyframes import KeyframeStore
from relnav.core.tracker import TrackerConfig, estimate_pose
from relnav.domain.se3 import SE3
from relnav.domain.types import Intrinsics, Keyframe

_reg = o3d.pipelines.registration


@dataclass(frozen=True)
class BackendConfig:
    loop_max_distance_m: float = 1.0
    loop_max_angle_deg: float = 45.0
    loop_exclude_recent: int = 10
    loop_min_inliers: int = 60
    loop_max_rmse_px: float = 2.0
    loop_max_candidates: int = 3
    edge_prune_threshold: float = 0.25


@dataclass(frozen=True)
class LoopClosure:
    new_id: int
    old_id: int
    inliers: int


class Backend:
    def __init__(self, cfg: BackendConfig, tracker_cfg: TrackerConfig, ratio: float) -> None:
        self.cfg = cfg
        self._tracker_cfg = tracker_cfg
        self._ratio = ratio
        self._graph = _reg.PoseGraph()
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

    def add_keyframe(
        self,
        keyframe: Keyframe,
        keyframes: KeyframeStore,
        odometry_inliers: int,
        intr: Intrinsics,
    ) -> LoopClosure | None:
        if keyframe.id != len(self._graph.nodes):
            raise ValueError(f"keyframe {keyframe.id} added out of order")
        self._graph.nodes.append(_reg.PoseGraphNode(keyframe.T_world_cam.matrix.copy()))
        if keyframe.id > 0:
            previous = keyframes[keyframe.id - 1]
            T_new_prev = keyframe.T_world_cam.inverse() @ previous.T_world_cam
            self._graph.edges.append(
                _reg.PoseGraphEdge(
                    previous.id,
                    keyframe.id,
                    T_new_prev.matrix.copy(),
                    np.eye(6) * max(odometry_inliers, 1),
                    uncertain=False,
                )
            )
        loop = self._detect_loop(keyframe, keyframes, intr)
        if loop is not None:
            self._optimize(keyframes)
        return loop

    def _detect_loop(
        self, keyframe: Keyframe, keyframes: KeyframeStore, intr: Intrinsics
    ) -> LoopClosure | None:
        recent = frozenset(
            range(max(0, keyframe.id - self.cfg.loop_exclude_recent), keyframe.id + 1)
        )
        candidates = keyframes.nearest(
            keyframe.T_world_cam,
            k=self.cfg.loop_max_candidates,
            max_distance_m=self.cfg.loop_max_distance_m,
            max_angle_deg=self.cfg.loop_max_angle_deg,
            exclude_ids=recent,
        )
        for candidate in candidates:
            m = match(keyframe.descriptors, candidate.descriptors, self._ratio)
            if len(m) < self.cfg.loop_min_inliers:
                continue
            # Pose of the new keyframe's camera in the candidate's frame: T_cand_new.
            result = estimate_pose(
                candidate.points_cam[m[:, 1]], keyframe.keypoints[m[:, 0]], intr, self._tracker_cfg
            )
            if (
                result.ok
                and result.T_world_cam is not None
                and result.inliers >= self.cfg.loop_min_inliers
                and result.rmse_px <= self.cfg.loop_max_rmse_px
            ):
                self._graph.edges.append(
                    _reg.PoseGraphEdge(
                        keyframe.id,
                        candidate.id,
                        result.T_world_cam.matrix.copy(),
                        np.eye(6) * result.inliers,
                        uncertain=True,
                    )
                )
                return LoopClosure(keyframe.id, candidate.id, result.inliers)
        return None

    def _optimize(self, keyframes: KeyframeStore) -> None:
        _reg.global_optimization(
            self._graph,
            _reg.GlobalOptimizationLevenbergMarquardt(),
            _reg.GlobalOptimizationConvergenceCriteria(),
            _reg.GlobalOptimizationOption(
                max_correspondence_distance=0.05,
                edge_prune_threshold=self.cfg.edge_prune_threshold,
                reference_node=0,
            ),
        )
        for node, keyframe in zip(self._graph.nodes, keyframes, strict=True):
            keyframe.T_world_cam = SE3(np.asarray(node.pose))
