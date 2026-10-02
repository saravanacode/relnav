"""ORB features on the infrared image, spread over a grid, with 3D points from depth."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from relnav.domain.geometry import deproject
from relnav.domain.types import Frame


@dataclass(frozen=True)
class FeatureConfig:
    n_features: int = 1000
    grid_cols: int = 8
    grid_rows: int = 6
    min_depth_m: float = 0.3
    max_depth_m: float = 4.0
    ratio: float = 0.8  # Lowe ratio test


@dataclass(frozen=True, eq=False)
class Features:
    keypoints: NDArray[np.float32]  # (N, 2) pixels
    descriptors: NDArray[np.uint8]  # (N, 32)
    points_cam: NDArray[np.float64]  # (N, 3) camera frame

    def __len__(self) -> int:
        return len(self.keypoints)

    @staticmethod
    def empty() -> Features:
        return Features(np.zeros((0, 2), np.float32), np.zeros((0, 32), np.uint8), np.zeros((0, 3)))


class FeatureExtractor:
    def __init__(self, cfg: FeatureConfig) -> None:
        self.cfg = cfg
        # Detect extra candidates; the grid keeps the strongest per cell.
        self._orb = cv2.ORB.create(nfeatures=cfg.n_features * 3, fastThreshold=10)

    def extract(self, frame: Frame) -> Features:
        keypoints = self._orb.detect(frame.ir, None)
        keypoints = self._spread_over_grid(keypoints, frame.ir.shape)
        if not keypoints:
            return Features.empty()
        keypoints, descriptors = self._orb.compute(frame.ir, keypoints)
        if descriptors is None or len(keypoints) == 0:
            return Features.empty()
        uv = np.array([k.pt for k in keypoints], dtype=np.float32)
        h, w = frame.depth_m.shape
        ui = np.clip(np.rint(uv[:, 0]).astype(int), 0, w - 1)
        vi = np.clip(np.rint(uv[:, 1]).astype(int), 0, h - 1)
        z = frame.depth_m[vi, ui].astype(np.float64)
        keep = (z >= self.cfg.min_depth_m) & (z <= self.cfg.max_depth_m)
        points = deproject(uv[keep], z[keep], frame.intrinsics)
        return Features(uv[keep], np.asarray(descriptors, dtype=np.uint8)[keep], points)

    def _spread_over_grid(
        self, keypoints: Sequence[cv2.KeyPoint], shape: tuple[int, ...]
    ) -> list[cv2.KeyPoint]:
        h, w = shape[:2]
        cols, rows = self.cfg.grid_cols, self.cfg.grid_rows
        per_cell = max(1, self.cfg.n_features // (cols * rows))
        cells: dict[tuple[int, int], list[cv2.KeyPoint]] = {}
        for k in keypoints:
            cell = (min(int(k.pt[0] * cols / w), cols - 1), min(int(k.pt[1] * rows / h), rows - 1))
            cells.setdefault(cell, []).append(k)
        spread: list[cv2.KeyPoint] = []
        for cell in sorted(cells):
            spread.extend(sorted(cells[cell], key=lambda k: -k.response)[:per_cell])
        return spread


def match(
    desc_query: NDArray[np.uint8], desc_train: NDArray[np.uint8], ratio: float
) -> NDArray[np.int64]:
    """Hamming kNN matching with Lowe's ratio test; each train feature used at most once.

    Returns (M, 2) rows of [query_index, train_index], sorted by query index.
    """
    if len(desc_query) < 2 or len(desc_train) < 2:
        return np.zeros((0, 2), np.int64)
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(desc_query, desc_train, k=2)
    best: dict[int, tuple[float, int]] = {}
    for pair in pairs:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            previous = best.get(m.trainIdx)
            if previous is None or m.distance < previous[0]:
                best[m.trainIdx] = (m.distance, m.queryIdx)
    rows = sorted((query, train) for train, (_, query) in best.items())
    return np.array(rows, dtype=np.int64).reshape(-1, 2)
