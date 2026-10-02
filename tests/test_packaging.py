"""Every source file must be committable: a .gitignore pattern must never hide code."""

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_no_source_or_test_file_is_git_ignored():
    files = [
        str(p.relative_to(ROOT))
        for d in ("src", "tests", "config", "docs")
        for p in (ROOT / d).rglob("*")
        if p.is_file() and "__pycache__" not in p.parts
    ]
    result = subprocess.run(
        ["git", "check-ignore", "--no-index", *files], cwd=ROOT, capture_output=True, text=True
    )
    ignored = result.stdout.split()
    assert ignored == [], f"git ignores these files: {ignored}"
