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


def test_doc_commands_run_through_uv():
    """Project rule: commands in docs use `uv run`, never bare python or pip."""
    docs = [ROOT / "README.md", *(ROOT / "docs").glob("*.md"), *(ROOT / "docs/adr").glob("*.md")]
    bare = re.compile(r"(?<!uv run )\b(python3?|pip3?)\s+(-c|-m|install)\b")
    offenders = [
        f"{doc.relative_to(ROOT)}: {line.strip()}"
        for doc in docs
        for line in doc.read_text().splitlines()
        if bare.search(line)
    ]
    assert offenders == []
