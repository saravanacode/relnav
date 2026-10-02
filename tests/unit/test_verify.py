import json

import pytest

from relnav.app.verify import check_pairs, load_pins, load_truth


def write_pins(tmp_path, pins):
    (tmp_path / "pins.json").write_text(
        json.dumps({"pins": [{"label": k, "position_m": v} for k, v in pins.items()]})
    )


def test_pairs_pass_and_fail(tmp_path):
    write_pins(tmp_path, {"DOOR": [0.0, 0.0, 2.0], "CHAIR": [2.39, 0.0, 2.0]})
    truth = tmp_path / "truth.yaml"
    truth.write_text("pairs:\n  - {a: DOOR, b: CHAIR, distance_m: 2.43}\n")
    (check,) = check_pairs(load_pins(tmp_path), load_truth(truth))
    assert check.measured_m == pytest.approx(2.39)
    assert check.error_cm == pytest.approx(-4.0)
    assert not check.passed(3.0)
    assert check.passed(4.5)


def test_missing_label_fails_without_crashing(tmp_path):
    """Review focus 3."""
    write_pins(tmp_path, {"DOOR": [0.0, 0.0, 2.0]})
    (check,) = check_pairs(load_pins(tmp_path), [("DOOR", "WINDOW", 1.0)])
    assert check.measured_m is None and check.error_cm is None
    assert not check.passed(3.0)


def test_malformed_truth_is_a_clear_error(tmp_path):
    truth = tmp_path / "truth.yaml"
    truth.write_text("pairs:\n  - {a: DOOR, distance_m: 2.0}\n")
    with pytest.raises(ValueError, match="pair 1"):
        load_truth(truth)
