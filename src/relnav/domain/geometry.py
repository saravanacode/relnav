"""Pinhole camera geometry: pixels <-> 3D points, depth sampling, bearings."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from relnav.domain.types import Intrinsics


def deproject(uv: ArrayLike, depth: ArrayLike, intr: Intrinsics) -> NDArray[np.float64]:
    """Pixels (N, 2) with depths (N,) in meters -> camera-frame points (N, 3)."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    z = np.asarray(depth, dtype=np.float64).reshape(-1)
    x = (uv[:, 0] - intr.cx) * z / intr.fx
    y = (uv[:, 1] - intr.cy) * z / intr.fy
    return np.stack([x, y, z], axis=1)


def project(points_cam: ArrayLike, intr: Intrinsics) -> NDArray[np.float64]:
    """Camera-frame points (N, 3) -> pixels (N, 2)."""
    p = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    u = intr.fx * p[:, 0] / p[:, 2] + intr.cx
    v = intr.fy * p[:, 1] / p[:, 2] + intr.cy
    return np.stack([u, v], axis=1)


def depth_patch_median(
    depth_m: NDArray[np.float32], u: int, v: int, half: int, min_valid_fraction: float
) -> float | None:
    """Median of valid depth in a (2*half+1)^2 patch around (u, v), clipped to the image.

    Returns None when fewer than ``min_valid_fraction`` of the patch pixels have depth.
    """
    h, w = depth_m.shape
    patch = depth_m[
        max(v - half, 0) : min(v + half + 1, h), max(u - half, 0) : min(u + half + 1, w)
    ]
    valid = patch[patch > 0]
    if patch.size == 0 or valid.size < min_valid_fraction * patch.size:
        return None
    return float(np.median(valid))


def bearing(point_cam: ArrayLike) -> tuple[float, float, float]:
    """Distance, azimuth (positive right) and elevation (positive up) of a camera-frame point."""
    x, y, z = np.asarray(point_cam, dtype=np.float64).reshape(3)
    distance = float(np.sqrt(x * x + y * y + z * z))
    azimuth = float(np.degrees(np.arctan2(x, z)))
    elevation = float(np.degrees(np.arctan2(-y, np.hypot(x, z))))
    return distance, azimuth, elevation


def sparse_cloud(
    depth_m: NDArray[np.float32], intr: Intrinsics, stride: int
) -> NDArray[np.float32]:
    """Every ``stride``-th pixel with depth, deprojected to camera-frame points (M, 3)."""
    vs, us = np.mgrid[0 : depth_m.shape[0] : stride, 0 : depth_m.shape[1] : stride]
    z = depth_m[vs, us].reshape(-1)
    keep = z > 0
    uv = np.stack([us.reshape(-1)[keep], vs.reshape(-1)[keep]], axis=1)
    return deproject(uv, z[keep], intr).astype(np.float32)
