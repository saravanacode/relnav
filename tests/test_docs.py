"""Keep the requirement table in step with the spec and the tests."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_every_spec_requirement_is_traced():
    spec = (ROOT / "docs/specs/2026-10-02-relnav-design.md").read_text()
    table = (ROOT / "docs/requirements.md").read_text()
    for req in sorted(set(re.findall(r"REQ-[A-Z]+-\d+", spec))):
        assert req in table, f"{req} missing from docs/requirements.md"


def test_traced_tests_exist():
    table = (ROOT / "docs/requirements.md").read_text()
    for path, name in re.findall(r"`(tests/[\w/]+\.py)::(\w+)`", table):
        assert f"def {name}(" in (ROOT / path).read_text(), f"{path}::{name} not found"
