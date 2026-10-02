"""Synthetic textured room for hardware-free tests.

Renders what a D435i-like camera sees (IR intensity + metric depth) at any pose, using
Open3D ray casting. Surfaces get blocky value-noise texture so ORB finds corners.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d

from relnav.domain.se3 import SE3
from relnav.domain.types import Frame, Intrinsics

SYN_INTRINSICS = Intrinsics(width=848, height=480, fx=420.0, fy=420.0, cx=424.0, cy=240.0)
SMALL_INTRINSICS = Intrinsics(width=424, height=240, fx=210.0, fy=210.0, cx=212.0, cy=120.0)
FRAME_PERIOD_NS = 33_333_333

_TABLE = np.random.default_rng(12345).random(1 << 16)
# (size, min corner) in OpenCV axes (y down). Camera starts at the origin looking down +z.
_ROOM = ((4.0, 2.7, 5.0), (-2.0, -1.5, -2.0))
_FURNITURE = (
    ((0.8, 0.75, 0.6), (-1.6, 0.45, 1.8)),
    ((0.5, 1.2, 0.5), (1.2, 0.0, 2.2)),
    ((1.2, 0.4, 0.6), (-0.6, 0.8, -1.8)),
)


def _noise(points: np.ndarray, cell: float) -> np.ndarray:
    """Blocky value noise: one random value per cubic cell of size ``cell`` meters."""
    i = np.floor(points / cell).astype(np.int64)
    h = (i[:, 0] * 73856093) ^ (i[:, 1] * 19349663) ^ (i[:, 2] * 83492791)
    return _TABLE[h & 0xFFFF]


class SyntheticRoom:
    def __init__(self) -> None:
        self._scene = o3d.t.geometry.RaycastingScene()
        for size, origin in (_ROOM, *_FURNITURE):
            box = o3d.geometry.TriangleMesh.create_box(*size)
            box.translate(origin)
            self._scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(box))

    def render(
        self,
        T_world_cam: SE3,
        index: int = 0,
        intrinsics: Intrinsics = SYN_INTRINSICS,
        depth_noise: bool = True,
    ) -> Frame:
        w, h = intrinsics.width, intrinsics.height
        T_cam_world = T_world_cam.inverse()
        rays = self._scene.create_rays_pinhole(
            o3d.core.Tensor(intrinsics.K),  # pyright: ignore[reportCallIssue, reportArgumentType]
            o3d.core.Tensor(T_cam_world.matrix),  # pyright: ignore[reportCallIssue, reportArgumentType]
            w,
            h,
        )
        t_hit = self._scene.cast_rays(rays)["t_hit"].numpy().reshape(-1)
        r = rays.numpy().reshape(-1, 6)
        hit_ok = np.isfinite(t_hit)
        hit = r[:, :3] + r[:, 3:] * np.where(hit_ok, t_hit, 0.0)[:, None]
        z = T_cam_world.apply(hit)[:, 2]
        if depth_noise:
            # D400-like error that grows with distance squared; seeded for determinism.
            z = z + np.random.default_rng(index).normal(0.0, 1.0, z.shape) * 0.001 * z**2
        valid = hit_ok & (z > 0.3) & (z < 4.0)
        depth = np.where(valid, z, 0.0).astype(np.float32).reshape(h, w)
        texture = 0.6 * _noise(hit, 0.08) + 0.4 * _noise(hit, 0.025)
        ir = np.where(hit_ok, 40 + 200 * texture, 0).astype(np.uint8).reshape(h, w)
        return Frame(
            index=index,
            timestamp_ns=index * FRAME_PERIOD_NS,
            ir=ir,
            depth_m=depth,
            intrinsics=intrinsics,
        )


def pose_xz_yaw(x: float, z: float, yaw_deg: float) -> SE3:
    """Camera at (x, 0, z) turned by ``yaw_deg`` about +y (positive = looks right)."""
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return SE3.from_rt(R, [x, 0.0, z])


def loop_trajectory(
    n_frames: int = 150, radius: float = 0.6, yaw_amplitude_deg: float = 30.0
) -> list[SE3]:
    """A horizontal circle that starts at the origin and comes back, looking around."""
    poses = []
    for i in range(n_frames):
        a = 2.0 * np.pi * i / n_frames
        x, z = radius * np.sin(a), radius * (1.0 - np.cos(a))
        poses.append(pose_xz_yaw(x, z, yaw_amplitude_deg * np.sin(a)))
    return poses


def blank_frame(index: int, intrinsics: Intrinsics = SYN_INTRINSICS) -> Frame:
    """What the camera sees with the lens covered: no texture and no depth."""
    shape = (intrinsics.height, intrinsics.width)
    return Frame(
        index=index,
        timestamp_ns=index * FRAME_PERIOD_NS,
        ir=np.zeros(shape, np.uint8),
        depth_m=np.zeros(shape, np.float32),
        intrinsics=intrinsics,
    )
