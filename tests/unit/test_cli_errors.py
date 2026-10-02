"""The CLI reports camera and input problems as messages with an exit code, not tracebacks."""

from typer.testing import CliRunner

import relnav.adapters.realsense.source as realsense
from relnav.app.cli import app
from relnav.ports.errors import FrameSourceError

runner = CliRunner()


def test_camera_that_will_not_start_is_a_clear_error(monkeypatch):
    def broken(cfg=None):
        raise FrameSourceError("camera did not start after 4 attempts: failed to set power state")

    monkeypatch.setattr(realsense, "RealSenseFrameSource", broken)
    for args in (["live", "--no-hud", "--no-rerun"], ["record", "x.mcap", "--no-hud"]):
        result = runner.invoke(app, args)
        assert result.exit_code == 1, result.output
        assert "camera did not start" in result.output
        assert not isinstance(result.exception, FrameSourceError)
