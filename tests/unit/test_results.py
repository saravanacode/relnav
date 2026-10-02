import json

import numpy as np
import pytest

from relnav.adapters.results.writer import TimingCollector, write_results, write_tum
from relnav.domain.se3 import SE3
from relnav.domain.types import Frame, Intrinsics, WorldState

INTR = Intrinsics(4, 3, 2.0, 2.0, 2.0, 1.5)


def state(total_ms: float) -> WorldState:
    frame = Frame(0, 0, np.zeros((3, 4), np.uint8), np.zeros((3, 4), np.float32), INTR)
    return WorldState(
        frame=frame,
        status=WorldState.preview(frame, "").status,
        timings_ms={"total_ms": total_ms, "features_ms": total_ms / 2},
    )


def test_tum_format(tmp_path):
    path = tmp_path / "trajectory.tum"
    turned = SE3.from_rvec_tvec([0.0, 0.0, np.pi / 2], [1.0, 2.0, 3.0])
    write_tum(path, [(0, SE3.identity()), (1_500_000_000, turned)])
    lines = path.read_text().splitlines()
    assert lines[0].startswith("#")
    assert lines[1].split() == [
        "0.000000000",
        "0.000000",
        "0.000000",
        "0.000000",
        "0.000000",
        "0.000000",
        "0.000000",
        "1.000000",
    ]
    values = [float(x) for x in lines[2].split()]
    assert values[:4] == [1.5, 1.0, 2.0, 3.0]
    assert values[4:] == pytest.approx([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], abs=1e-6)


def test_timing_summary():
    collector = TimingCollector()
    for ms in (10.0, 20.0, 30.0, 40.0):
        collector.publish(state(ms))
    summary = collector.summary()
    assert summary["frames"] == 4
    assert summary["total_ms"]["mean"] == pytest.approx(25.0)
    assert summary["total_ms"]["max"] == pytest.approx(40.0)
    assert summary["fps"] == pytest.approx(40.0)
    assert TimingCollector().summary() == {"frames": 0}


def test_write_results_creates_all_files(tmp_path):
    out = tmp_path / "run"
    written = write_results(
        out,
        trajectory=[(0, SE3.identity())],
        pins=[("DOOR", np.array([1.0, 0.0, 2.0]))],
        timing={"frames": 1},
        summary={"frames": 1, "stop_reason": "end of stream"},
    )
    assert sorted(p.name for p in written) == [
        "pins.json",
        "summary.json",
        "timing.json",
        "trajectory.tum",
    ]
    pins = json.loads((out / "pins.json").read_text())
    assert pins == {"pins": [{"label": "DOOR", "position_m": [1.0, 0.0, 2.0]}]}
