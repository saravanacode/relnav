import numpy as np

from relnav.core.features import Features
from relnav.core.keyframes import KeyframePolicy, KeyframeStore, viewing_angle_deg
from relnav.domain.se3 import SE3
from tests.support.synthetic import pose_xz_yaw

EMPTY_CLOUD = np.zeros((0, 3), np.float32)


def features(n: int) -> Features:
    return Features(np.zeros((n, 2), np.float32), np.zeros((n, 32), np.uint8), np.zeros((n, 3)))


def store_with(poses: list[SE3], n_features: int = 200) -> KeyframeStore:
    store = KeyframeStore()
    for i, T in enumerate(poses):
        store.add(i, T, features(n_features), EMPTY_CLOUD)
    return store


def test_ids_are_sequential_and_latest_is_last():
    store = store_with([SE3.identity(), pose_xz_yaw(0.2, 0, 0)])
    assert [kf.id for kf in store] == [0, 1]
    assert len(store) == 2 and store[1].frame_index == 1
    assert store.latest is store[1]
    assert KeyframeStore().latest is None


def test_policy_triggers_on_each_criterion():
    policy = KeyframePolicy()
    reference = store_with([SE3.identity()])[0]
    small_move = pose_xz_yaw(0.05, 0.05, 5)
    assert not policy.needs_keyframe(reference, small_move, inliers=150, baseline_inliers=200)
    assert policy.needs_keyframe(reference, pose_xz_yaw(0.16, 0.0, 0), 150, 200)
    assert policy.needs_keyframe(reference, pose_xz_yaw(0.0, 0.0, 16), 150, 200)
    assert policy.needs_keyframe(reference, SE3.identity(), inliers=99, baseline_inliers=120)
    # Overlap halved: fewer than 50% of the inliers this keyframe had when first tracked.
    assert policy.needs_keyframe(reference, SE3.identity(), inliers=140, baseline_inliers=300)


def test_policy_ignores_how_many_raw_features_the_keyframe_has():
    """Only ~25% of a keyframe's features survive matching even without motion, so the
    raw feature count must not drive keyframe creation."""
    policy = KeyframePolicy()
    big = store_with([SE3.identity()], n_features=900)[0]
    assert not policy.needs_keyframe(big, pose_xz_yaw(0.01, 0.0, 0.6), 200, baseline_inliers=220)


def test_viewing_angle():
    assert np.isclose(viewing_angle_deg(SE3.identity(), pose_xz_yaw(0, 0, 30)), 30.0)


def test_nearest_filters_and_sorts():
    store = store_with(
        [
            pose_xz_yaw(0.0, 0.0, 0),  # 0: at query
            pose_xz_yaw(0.5, 0.0, 0),  # 1: 0.5 m away
            pose_xz_yaw(0.2, 0.0, 90),  # 2: close but facing sideways
            pose_xz_yaw(2.0, 0.0, 0),  # 3: too far
        ]
    )
    query = pose_xz_yaw(0.05, 0.0, 0)
    ids = [kf.id for kf in store.nearest(query, max_distance_m=1.0, max_angle_deg=45)]
    assert ids == [0, 1]
    assert [kf.id for kf in store.nearest(query, k=2)] == [0, 2]
    assert [kf.id for kf in store.nearest(query, k=1, exclude_ids=frozenset({0}))] == [2]
