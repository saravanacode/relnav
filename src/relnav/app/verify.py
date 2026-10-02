"""Compare measured pin-to-pin distances with tape measurements (REQ-ACC-01)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from numpy.typing import NDArray


@dataclass(frozen=True)
class PairCheck:
    a: str
    b: str
    truth_m: float
    measured_m: float | None  # None when a label is missing from pins.json

    @property
    def error_cm(self) -> float | None:
        if self.measured_m is None:
            return None
        return (self.measured_m - self.truth_m) * 100.0

    def passed(self, tolerance_cm: float) -> bool:
        error = self.error_cm
        return error is not None and abs(error) < tolerance_cm


def load_pins(results_dir: Path) -> dict[str, NDArray[np.float64]]:
    data = json.loads((results_dir / "pins.json").read_text())
    return {row["label"]: np.array(row["position_m"], dtype=np.float64) for row in data["pins"]}


def load_truth(path: Path) -> list[tuple[str, str, float]]:
    data = yaml.safe_load(path.read_text()) or {}
    pairs = data.get("pairs")
    if not isinstance(pairs, list) or not pairs:
        raise ValueError(f"{path}: expected a non-empty 'pairs' list")
    out = []
    for n, pair in enumerate(pairs, start=1):
        try:
            out.append((str(pair["a"]), str(pair["b"]), float(pair["distance_m"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{path}: pair {n} needs a, b and distance_m ({exc})") from exc
    return out


def check_pairs(
    pins: dict[str, NDArray[np.float64]], truth: list[tuple[str, str, float]]
) -> list[PairCheck]:
    checks = []
    for a, b, distance in truth:
        measured = None
        if a in pins and b in pins:
            measured = float(np.linalg.norm(pins[a] - pins[b]))
        checks.append(PairCheck(a, b, distance, measured))
    return checks
