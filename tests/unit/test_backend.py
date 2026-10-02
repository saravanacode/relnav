import numpy as np
import pytest

from relnav.core.backend import Backend, BackendConfig
from relnav.core.features import FeatureConfig, FeatureExtractor
from relnav.core.keyframes import KeyframeStore
from relnav.core.tracker import TrackerConfig
from relnav.domain.se3 import SE3
from tests.support.synthetic import SYN_INTRINSICS, SyntheticRoom, loop_trajectory, pose_xz_yaw

EMPTY_CLOUD = np.zeros((0, 3), np.float32)


@pytest.fixture(scope="module")
def room():
    return SyntheticRoom()


@pytest.fixture(scope="module")
def extractor():
    return FeatureExtractor(FeatureConfig())


def backend() -> Backend:
    return Backend(BackendConfig(), TrackerConfig(), ratio=0.8)


def test_loop_closure_pulls_drifted_keyframes_back(room, extractor):
    truth = [*loop_trajectory(16), SE3.identity()]  # 16 keyframes round the loop, then home
    # Odometry that drifts a little every keyframe, like real visual odometry.
    drift_step = SE3.from_rvec_tvec([0.0, np.radians(0.3), 0.0], [0.005, 0.0, 0.003])
    drifted, drift = [], SE3.identity()
    for i, T_true in enumerate(truth):
        drift = drift @ drift_step if i > 0 else drift
        drifted.append(drift @ T_true)
    store, graph, loops = KeyframeStore(), backend(), []
    for i, T_true in enumerate(truth):
        # Like the tracker, place each keyframe relative to the (possibly corrected) previous one.
        T_est = (
            SE3.identity()
            if i == 0
            else store[i - 1].T_world_cam @ (drifted[i - 1].inverse() @ drifted[i])
        )
        frame = room.render(T_true, index=i, depth_noise=False)
        kf = store.add(i, T_est, extractor.extract(frame), EMPTY_CLOUD)
        loop = graph.add_keyframe(kf, store, odometry_inliers=200, intr=SYN_INTRINSICS)
        if loop is not None:
            loops.append(loop)
    before = np.linalg.norm(drifted[-1].t - truth[-1].t)
    after = np.linalg.norm(store[16].T_world_cam.t - truth[-1].t)
    assert loops, "expected at least one loop closure"
    assert all(loop.inliers >= 60 for loop in loops)
    assert before > 0.05
    assert after < 0.5 * before
    assert store[0].T_world_cam.allclose(SE3.identity(), atol=1e-9)  # origin is fixed


def test_no_loop_on_a_straight_walk(room, extractor):
    store, graph = KeyframeStore(), backend()
    for i in range(3):
        T = pose_xz_yaw(0.0, 0.2 * i, 0.0)
        kf = store.add(i, T, extractor.extract(room.render(T, index=i)), EMPTY_CLOUD)
        assert graph.add_keyframe(kf, store, odometry_inliers=200, intr=SYN_INTRINSICS) is None
        assert store[i].T_world_cam.allclose(T)


def test_recent_keyframes_are_never_loop_candidates(room, extractor):
    store, graph = KeyframeStore(), backend()
    frame = room.render(SE3.identity(), depth_noise=False)
    features = extractor.extract(frame)
    for i in range(5):  # same view five times: all within the 10 most recent
        kf = store.add(i, SE3.identity(), features, EMPTY_CLOUD)
        assert graph.add_keyframe(kf, store, odometry_inliers=200, intr=SYN_INTRINSICS) is None
