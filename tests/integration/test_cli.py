import json

import pytest
from typer.testing import CliRunner

from relnav.adapters.mcap.recorder import McapRecorder
from relnav.app.cli import app
from relnav.domain.commands import DropPin
from tests.support.synthetic import SyntheticRoom, loop_trajectory

pytestmark = pytest.mark.slow
runner = CliRunner()


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    room = SyntheticRoom()
    path = tmp_path_factory.mktemp("rec") / "synthetic.mcap"
    recorder = McapRecorder(path)
    for i, T in enumerate(loop_trajectory(150)[:45]):
        frame = room.render(T, index=i)
        if i == 10:
            recorder.write_command(frame.timestamp_ns, DropPin(label="WALL"))
        recorder.write_frame(frame)
    recorder.close()
    return path


def replay(recording, results, name):
    args = ["replay", str(recording), "--no-hud", "--no-rerun", "--results", str(results)]
    return runner.invoke(app, [*args, "--name", name])


def test_replay_writes_results(recording, tmp_path):
    result = replay(recording, tmp_path, "run")
    assert result.exit_code == 0, result.output
    out = tmp_path / "run"
    for name in ("trajectory.tum", "pins.json", "timing.json", "summary.json"):
        assert (out / name).exists()
    pins = json.loads((out / "pins.json").read_text())["pins"]
    assert [p["label"] for p in pins] == ["WALL"]
    summary = json.loads((out / "summary.json").read_text())
    assert summary["frames"] == 45 and summary["stop_reason"] == "end of stream"
    assert summary["dropped_frames"] == 0  # replay never drops frames


def test_replay_is_deterministic(recording, tmp_path):
    """REQ-REP-01: the same recording replays to identical trajectory and pins."""
    for name in ("a", "b"):
        assert replay(recording, tmp_path, name).exit_code == 0
    for file in ("trajectory.tum", "pins.json"):
        assert (tmp_path / "a" / file).read_text() == (tmp_path / "b" / file).read_text()


def write_pins(directory, pins):
    directory.mkdir(parents=True, exist_ok=True)
    rows = [{"label": k, "position_m": v} for k, v in pins.items()]
    (directory / "pins.json").write_text(json.dumps({"pins": rows}))


def test_verify_passes_and_fails(tmp_path):
    write_pins(tmp_path / "run", {"DOOR": [0, 0, 2], "CHAIR": [2.42, 0, 2]})
    good = tmp_path / "good.yaml"
    good.write_text("pairs:\n  - {a: DOOR, b: CHAIR, distance_m: 2.43}\n")
    bad = tmp_path / "bad.yaml"
    bad.write_text("pairs:\n  - {a: DOOR, b: CHAIR, distance_m: 2.60}\n")
    ok = runner.invoke(app, ["verify", str(tmp_path / "run"), str(good)])
    assert ok.exit_code == 0, ok.output
    assert "PASS" in ok.output
    failed = runner.invoke(app, ["verify", str(tmp_path / "run"), str(bad)])
    assert failed.exit_code == 1 and "FAIL" in failed.output


def test_verify_reports_missing_label(tmp_path):
    """Review focus 3."""
    write_pins(tmp_path / "run", {"DOOR": [0, 0, 2]})
    truth = tmp_path / "t.yaml"
    truth.write_text("pairs:\n  - {a: DOOR, b: WINDOW, distance_m: 1.0}\n")
    result = runner.invoke(app, ["verify", str(tmp_path / "run"), str(truth)])
    assert result.exit_code == 1
    assert "MISSING" in result.output
    assert result.exception is None or isinstance(result.exception, SystemExit)
