"""Rigid-body transforms (SE(3)).

Naming: ``T_a_b`` maps points from frame b into frame a, so ``T_a_c = T_a_b @ T_b_c``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Mat = NDArray[np.float64]


def hat(w: ArrayLike) -> Mat:
    """Vector -> skew-symmetric matrix, so hat(a) @ b == cross(a, b)."""
    x, y, z = np.asarray(w, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_exp(rvec: ArrayLike) -> Mat:
    """Rotation vector (axis * angle in radians) -> rotation matrix (Rodrigues)."""
    r = np.asarray(rvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(r))
    K = hat(r)
    if theta < 1e-12:
        return np.eye(3) + K
    a = np.sin(theta) / theta
    b = (1.0 - np.cos(theta)) / theta**2
    return np.eye(3) + a * K + b * (K @ K)


def so3_log(R: ArrayLike) -> Mat:
    """Rotation matrix -> rotation vector."""
    R = np.asarray(R, dtype=np.float64)
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos))
    vee = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-8:
        return vee / 2.0
    if np.pi - theta < 1e-6:
        # Near 180 degrees sin(theta) ~ 0; take the axis from (R + I) / 2 = a a^T.
        B = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(B)))
        axis = B[:, i] / np.sqrt(B[i, i])
        return axis * theta
    return vee * (theta / (2.0 * np.sin(theta)))


@dataclass(frozen=True, eq=False)
class SE3:
    """4x4 homogeneous rigid transform. Immutable: the matrix is a read-only copy."""

    matrix: Mat

    def __post_init__(self) -> None:
        m = np.array(self.matrix, dtype=np.float64, copy=True)
        if m.shape != (4, 4):
            raise ValueError(f"SE3 needs a 4x4 matrix, got {m.shape}")
        m.flags.writeable = False
        object.__setattr__(self, "matrix", m)

    @staticmethod
    def identity() -> SE3:
        return SE3(np.eye(4))

    @staticmethod
    def from_rt(R: ArrayLike, t: ArrayLike) -> SE3:
        m = np.eye(4)
        m[:3, :3] = np.asarray(R, dtype=np.float64)
        m[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
        return SE3(m)

    @staticmethod
    def from_rvec_tvec(rvec: ArrayLike, tvec: ArrayLike) -> SE3:
        return SE3.from_rt(so3_exp(rvec), tvec)

    @property
    def R(self) -> Mat:
        return self.matrix[:3, :3]

    @property
    def t(self) -> Mat:
        return self.matrix[:3, 3]

    def inverse(self) -> SE3:
        Rt = self.R.T
        return SE3.from_rt(Rt, -Rt @ self.t)

    def __matmul__(self, other: SE3) -> SE3:
        return SE3(self.matrix @ other.matrix)

    def apply(self, points: ArrayLike) -> Mat:
        p = np.asarray(points, dtype=np.float64)
        out = p.reshape(-1, 3) @ self.R.T + self.t
        return out[0] if p.ndim == 1 else out

    def rvec_tvec(self) -> tuple[Mat, Mat]:
        return so3_log(self.R), self.t.copy()

    def translation_norm(self) -> float:
        return float(np.linalg.norm(self.t))

    def rotation_angle_deg(self) -> float:
        return float(np.degrees(np.linalg.norm(so3_log(self.R))))

    def quaternion_xyzw(self) -> Mat:
        R = self.R
        tr = float(np.trace(R))
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            q = [
                (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
                0.25 * s,
            ]
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q = [
                0.25 * s,
                (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s,
            ]
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q = [
                (R[0, 1] + R[1, 0]) / s,
                0.25 * s,
                (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s,
            ]
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q = [
                (R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s,
                0.25 * s,
                (R[1, 0] - R[0, 1]) / s,
            ]
        q = np.array(q)
        return q / np.linalg.norm(q)

    def allclose(self, other: SE3, atol: float = 1e-9) -> bool:
        return bool(np.allclose(self.matrix, other.matrix, atol=atol))
