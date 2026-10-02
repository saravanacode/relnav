import numpy as np
import pytest

from relnav.core.features import FeatureConfig, FeatureExtractor, Features
from relnav.core.tracker import MotionModel, Tracker, TrackerConfig, estimate_pose
from relnav.domain.geometry import project
from relnav.domain.se3 import SE3
from relnav.domain.types import Keyframe
from tests.support.synthetic import SYN_INTRINSICS, SyntheticRoom, blank_frame, pose_xz_yaw

CFG = TrackerConfig()


def keyframe_from(features: Features, T_world_cam: SE3, kf_id: int = 0) -> Keyframe:
    return Keyframe(
        id=kf_id,
        frame_index=kf_id,
        T_world_cam=T_world_cam,
        keypoints=features.keypoints,
        descriptors=features.descriptors,
        points_cam=features.points_cam,
        cloud_cam=np.zeros((0, 3), np.float32),
    )


def scene(n=200, seed=0):
    rng = np.random.default_rng(seed)
    points = rng.uniform([-1.5, -1.0, 2.0], [1.5, 1.0, 4.0], (n, 3))
    T_world_cam = SE3.from_rvec_tvec([0.02, -0.05, 0.01], [0.1, 0.02, -0.05])
    uv = project(T_world_cam.inverse().apply(points), SYN_INTRINSICS)
    return points, uv, T_world_cam


@pytest.fixture(scope="module")
def room():
    return SyntheticRoom()


@pytest.fixture(scope="module")
def extractor():
    return FeatureExtractor(FeatureConfig())


def test_estimate_pose_recovers_exact_pose():
    points, uv, T = scene()
    r = estimate_pose(points, uv, SYN_INTRINSICS, CFG)
    assert r.ok and r.inliers == 200 and r.rmse_px < 1e-3
    assert r.T_world_cam is not None and r.T_world_cam.allclose(T, atol=1e-6)


def test_estimate_pose_rejects_outliers():
    points, uv, T = scene()
    uv[:60] += np.random.default_rng(1).uniform(20, 60, (60, 2))
    r = estimate_pose(points, uv, SYN_INTRINSICS, CFG)
    assert r.ok and 135 <= r.inliers <= 145
    assert r.T_world_cam is not None and r.T_world_cam.allclose(T, atol=1e-4)


def test_estimate_pose_needs_six_points():
    points, uv, _ = scene()
    assert not estimate_pose(points[:5], uv[:5], SYN_INTRINSICS, CFG).ok


def test_bad_guess_falls_back_to_unguided_solve():
    points, uv, T = scene()
    guess = SE3.from_rvec_tvec([1.0, 2.0, 0.5], [3.0, 3.0, 3.0])
    r = estimate_pose(points, uv, SYN_INTRINSICS, CFG, guess=guess)
    assert r.ok and r.T_world_cam is not None and r.T_world_cam.allclose(T, atol=1e-6)


def test_tracker_follows_small_motion(room, extractor):
    A, B = SE3.identity(), pose_xz_yaw(0.05, 0.03, 3.0)
    reference = keyframe_from(extractor.extract(room.render(A, index=0)), A)
    r = Tracker(CFG, 0.8).track(
        extractor.extract(room.render(B, index=1)), reference, SYN_INTRINSICS, None
    )
    assert r.ok and r.T_world_cam is not None
    error = r.T_world_cam.inverse() @ B
    assert error.translation_norm() < 0.01
    assert error.rotation_angle_deg() < 0.5


def test_tracker_fails_on_blank_frame(room, extractor):
    reference = keyframe_from(extractor.extract(room.render(SE3.identity())), SE3.identity())
    r = Tracker(CFG, 0.8).track(extractor.extract(blank_frame(1)), reference, SYN_INTRINSICS, None)
    assert not r.ok and r.T_world_cam is None


def test_relocalize_picks_the_keyframe_that_sees_the_same_place(room, extractor):
    near = keyframe_from(extractor.extract(room.render(SE3.identity())), SE3.identity(), 0)
    behind = pose_xz_yaw(0.0, 0.0, 180.0)
    far = keyframe_from(extractor.extract(room.render(behind, index=1)), behind, 1)
    current = extractor.extract(room.render(pose_xz_yaw(0.04, 0.0, 2.0), index=2))
    found = Tracker(CFG, 0.8).relocalize(current, [far, near], SYN_INTRINSICS, 60)
    assert found is not None
    result, keyframe = found
    assert keyframe.id == 0 and result.inliers >= 60


def test_motion_model_constant_velocity():
    model = MotionModel()
    assert model.predict() is None
    model.update(SE3.identity())
    predicted = model.predict()
    assert predicted is not None and predicted.allclose(SE3.identity())
    step = SE3.from_rvec_tvec([0.0, 0.05, 0.0], [0.02, 0.0, 0.01])
    model.update(step)
    predicted = model.predict()
    assert predicted is not None and predicted.allclose(step @ step)
    model.reset()
    assert model.predict() is None
