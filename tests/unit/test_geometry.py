import numpy as np
import pytest

from relnav.domain.geometry import bearing, deproject, depth_patch_median, project, sparse_cloud
from relnav.domain.types import Intrinsics

INTR = Intrinsics(width=640, height=480, fx=400.0, fy=410.0, cx=320.0, cy=240.0)


def test_deproject_then_project_roundtrips():
    uv = np.array([[320.0, 240.0], [100.5, 50.25], [600.0, 470.0]])
    z = np.array([1.0, 2.5, 3.9])
    points = deproject(uv, z, INTR)
    assert np.allclose(points[0], [0.0, 0.0, 1.0])
    assert np.allclose(project(points, INTR), uv)


def test_patch_median_ignores_holes():
    depth = np.full((480, 640), 2.0, np.float32)
    depth[238:243, 318:323] = 0.0  # 25 of 81 pixels missing (31% holes)
    depth[240, 324] = 9.0  # one outlier
    assert depth_patch_median(depth, 320, 240, half=4, min_valid_fraction=0.3) == 2.0


def test_patch_median_needs_enough_valid_depth():
    depth = np.zeros((480, 640), np.float32)
    depth[240, 320] = 2.0
    assert depth_patch_median(depth, 320, 240, half=4, min_valid_fraction=0.3) is None


def test_patch_median_clips_at_image_border():
    depth = np.full((480, 640), 1.5, np.float32)
    assert depth_patch_median(depth, 0, 0, half=4, min_valid_fraction=0.3) == 1.5


@pytest.mark.parametrize(
    ("point", "expected"),
    [
        ((0.0, 0.0, 2.0), (2.0, 0.0, 0.0)),
        ((1.0, 0.0, 1.0), (np.sqrt(2), 45.0, 0.0)),
        ((-1.0, 0.0, 1.0), (np.sqrt(2), -45.0, 0.0)),
        ((0.0, -1.0, 1.0), (np.sqrt(2), 0.0, 45.0)),
    ],
)
def test_bearing(point, expected):
    assert np.allclose(bearing(np.array(point)), expected)


def test_sparse_cloud_keeps_only_valid_pixels():
    depth = np.full((480, 640), 2.0, np.float32)
    depth[:, :320] = 0.0
    cloud = sparse_cloud(depth, INTR, stride=8)
    assert cloud.dtype == np.float32
    assert cloud.shape == (60 * 40, 3)
    assert np.allclose(cloud[:, 2], 2.0)
    assert np.all(cloud[:, 0] >= 0.0)
