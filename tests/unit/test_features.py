import dataclasses

import numpy as np
import pytest

from relnav.core.features import FeatureConfig, FeatureExtractor, Features, match
from relnav.domain.geometry import project
from relnav.domain.se3 import SE3
from tests.support.synthetic import SYN_INTRINSICS, SyntheticRoom, blank_frame, pose_xz_yaw


@pytest.fixture(scope="module")
def room():
    return SyntheticRoom()


@pytest.fixture(scope="module")
def extractor():
    return FeatureExtractor(FeatureConfig())


def test_features_spread_over_grid_with_valid_depth(room, extractor):
    frame = room.render(SE3.identity(), depth_noise=False)
    f = extractor.extract(frame)
    assert len(f) >= 400
    cols = (f.keypoints[:, 0] * 8 / frame.intrinsics.width).astype(int)
    rows = (f.keypoints[:, 1] * 6 / frame.intrinsics.height).astype(int)
    assert len(set(zip(cols.tolist(), rows.tolist(), strict=True))) >= 40
    assert np.all((f.points_cam[:, 2] >= 0.3) & (f.points_cam[:, 2] <= 4.0))
    assert f.descriptors.shape == (len(f), 32) and f.descriptors.dtype == np.uint8
    assert f.keypoints.dtype == np.float32 and f.points_cam.dtype == np.float64


def test_keypoints_without_depth_are_dropped(room, extractor):
    frame = room.render(SE3.identity(), depth_noise=False)
    depth = frame.depth_m.copy()
    depth[:, : frame.intrinsics.width // 2] = 0.0
    f = extractor.extract(dataclasses.replace(frame, depth_m=depth))
    assert len(f) > 0
    assert np.all(f.keypoints[:, 0] >= frame.intrinsics.width / 2 - 0.5)


def test_blank_frame_gives_empty_features(extractor):
    f = extractor.extract(blank_frame(0))
    assert len(f) == 0
    assert f.keypoints.shape == (0, 2)
    assert f.descriptors.shape == (0, 32)
    assert f.points_cam.shape == (0, 3)


def test_match_with_itself_pairs_each_feature_with_itself(room, extractor):
    f = extractor.extract(room.render(SE3.identity()))
    m = match(f.descriptors, f.descriptors, 0.8)
    assert len(m) >= 0.95 * len(f)
    assert np.mean(m[:, 0] == m[:, 1]) > 0.99


def test_matches_are_geometrically_correct_after_motion(room, extractor):
    A, B = SE3.identity(), pose_xz_yaw(0.05, 0.03, 3.0)
    fa = extractor.extract(room.render(A, index=0))
    fb = extractor.extract(room.render(B, index=1))
    m = match(fb.descriptors, fa.descriptors, 0.8)  # query: current frame, train: reference
    assert len(m) >= 150
    points_in_b = (B.inverse() @ A).apply(fa.points_cam[m[:, 1]])
    errors = np.linalg.norm(project(points_in_b, SYN_INTRINSICS) - fb.keypoints[m[:, 0]], axis=1)
    assert np.mean(errors < 3.0) >= 0.85


def test_match_handles_too_few_descriptors():
    empty = Features.empty()
    assert match(empty.descriptors, empty.descriptors, 0.8).shape == (0, 2)
