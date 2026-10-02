"""Write run results. trajectory.tum is readable by the `evo` evaluation tool."""

from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from relnav.adapters.fs import give_to_sudo_user, make_dirs_for_user
from relnav.domain.se3 import SE3
from relnav.domain.types import WorldState


def write_tum(path: Path, trajectory: Sequence[tuple[int, SE3]]) -> None:
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for timestamp_ns, T in trajectory:
        values = [*T.t, *T.quaternion_xyzw()]
        lines.append(f"{timestamp_ns / 1e9:.9f} " + " ".join(f"{v:.6f}" for v in values))
    path.write_text("\n".join(lines) + "\n")


class TimingCollector:
    """A StateSink that keeps per-frame timings for timing.json."""

    def __init__(self) -> None:
        self._rows: list[dict[str, float]] = []

    def publish(self, state: WorldState) -> None:
        self._rows.append(dict(state.timings_ms))

    def close(self) -> None:
        pass

    def summary(self) -> dict:
        if not self._rows:
            return {"frames": 0}
        out: dict = {"frames": len(self._rows)}
        for key in sorted({k for row in self._rows for k in row}):
            values = np.array([row[key] for row in self._rows if key in row])
            out[key] = {
                "mean": float(values.mean()),
                "p95": float(np.percentile(values, 95)),
                "max": float(values.max()),
            }
        if "total_ms" in out:
            out["fps"] = 1000.0 / out["total_ms"]["mean"]
        return out


def write_results(
    out_dir: Path,
    *,
    trajectory: Sequence[tuple[int, SE3]],
    pins: Sequence[tuple[str, NDArray[np.float64]]],
    timing: dict,
    summary: dict,
) -> list[Path]:
    make_dirs_for_user(out_dir)
    paths = {
        "trajectory": out_dir / "trajectory.tum",
        "pins": out_dir / "pins.json",
        "timing": out_dir / "timing.json",
        "summary": out_dir / "summary.json",
    }
    write_tum(paths["trajectory"], trajectory)
    pin_rows = [{"label": label, "position_m": [float(v) for v in xyz]} for label, xyz in pins]
    paths["pins"].write_text(json.dumps({"pins": pin_rows}, indent=2) + "\n")
    paths["timing"].write_text(json.dumps(timing, indent=2) + "\n")
    paths["summary"].write_text(json.dumps(summary, indent=2) + "\n")
    for path in paths.values():
        give_to_sudo_user(path)
    return list(paths.values())
