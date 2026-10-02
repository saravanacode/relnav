from pathlib import Path

import pytest
from pydantic import ValidationError

from relnav.app.config import load_config


def test_defaults_without_a_file():
    cfg = load_config(None)
    assert cfg.camera.width == 848
    assert cfg.slam.features.n_features == 1000
    assert cfg.rerun_url is None


def test_yaml_overrides_nested_values(tmp_path):
    path = tmp_path / "c.yaml"
    path.write_text(
        "camera:\n  emitter_enabled: true\n"
        "slam:\n  pins:\n    labels: [DOOR, CHAIR]\n  tracker:\n    min_inliers: 30\n"
    )
    cfg = load_config(path)
    assert cfg.camera.emitter_enabled is True
    assert cfg.slam.pins.labels == ("DOOR", "CHAIR")
    assert cfg.slam.tracker.min_inliers == 30


def test_typos_are_rejected(tmp_path):
    path = tmp_path / "c.yaml"
    path.write_text("slam:\n  trackr:\n    min_inliers: 30\n")
    with pytest.raises(ValidationError):
        load_config(path)


def test_repository_default_config_loads():
    cfg = load_config(Path("config/default.yaml"))
    assert cfg.slam.pins.labels == ("DOOR", "CHAIR", "LAPTOP")


def test_usb2_config_uses_a_mode_usb2_can_stream():
    """On a USB 2 link the D435i offers 640x480 depth + infrared at up to 30 fps; 15 is safe."""
    cfg = load_config(Path("config/usb2.yaml"))
    assert (cfg.camera.width, cfg.camera.height, cfg.camera.fps) == (640, 480, 15)
    assert cfg.camera.emitter_enabled is False
    assert cfg.slam.pins.labels == ("DOOR", "CHAIR", "LAPTOP")
