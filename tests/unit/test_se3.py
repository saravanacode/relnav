import numpy as np
import pytest
from hypothesis import given
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays

from relnav.domain.se3 import SE3, so3_exp, so3_log

# |rvec| <= sqrt(3) * 1.8 < pi, so the log map is unique.
rvecs = arrays(np.float64, 3, elements=st.floats(-1.8, 1.8))
tvecs = arrays(np.float64, 3, elements=st.floats(-10, 10))
poses = st.builds(SE3.from_rvec_tvec, rvecs, tvecs)
points = arrays(np.float64, (5, 3), elements=st.floats(-10, 10))


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@given(poses)
def test_inverse_composes_to_identity(T):
    assert (T @ T.inverse()).allclose(SE3.identity(), atol=1e-9)


@given(poses, poses, points)
def test_composition_matches_sequential_apply(A, B, p):
    assert np.allclose((A @ B).apply(p), A.apply(B.apply(p)), atol=1e-9)


@given(rvecs)
def test_rotation_is_orthonormal(r):
    R = so3_exp(r)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)


@given(rvecs)
def test_exp_log_roundtrip(r):
    assert np.allclose(so3_log(so3_exp(r)), r, atol=1e-9)


@given(rvecs)
def test_quaternion_matches_rotation(r):
    T = SE3.from_rvec_tvec(r, np.zeros(3))
    q = T.quaternion_xyzw()
    assert np.isclose(np.linalg.norm(q), 1.0)
    assert np.allclose(quat_to_matrix(q), T.R, atol=1e-9)


def test_log_near_pi_roundtrips():
    R = so3_exp([np.pi - 1e-8, 0.0, 0.0])
    assert np.allclose(so3_exp(so3_log(R)), R, atol=1e-6)


def test_known_rotation_maps_x_to_y():
    T = SE3.from_rvec_tvec([0.0, 0.0, np.pi / 2], [1.0, 0.0, 0.0])
    assert np.allclose(T.apply([1.0, 0.0, 0.0]), [1.0, 1.0, 0.0])


def test_apply_keeps_shape():
    T = SE3.identity()
    assert T.apply([1.0, 2.0, 3.0]).shape == (3,)
    assert T.apply(np.zeros((4, 3))).shape == (4, 3)


def test_angle_and_translation_norm():
    T = SE3.from_rvec_tvec([0.0, np.radians(30.0), 0.0], [3.0, 4.0, 0.0])
    assert np.isclose(T.rotation_angle_deg(), 30.0)
    assert np.isclose(T.translation_norm(), 5.0)


def test_identity_quaternion():
    assert np.allclose(SE3.identity().quaternion_xyzw(), [0, 0, 0, 1])


def test_matrix_is_read_only_and_copied():
    source = np.eye(4)
    T = SE3(source)
    source[0, 3] = 5.0
    assert T.t[0] == 0.0
    with pytest.raises(ValueError):
        T.matrix[0, 0] = 2.0


def test_rejects_wrong_shape():
    with pytest.raises(ValueError):
        SE3(np.eye(3))
