"""REQ-ARCH-01: the core stays free of hardware, UI, file-format and CLI code."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_layer_packages_import():
    import relnav.adapters
    import relnav.app
    import relnav.core
    import relnav.domain
    import relnav.ports  # noqa: F401


def test_import_contracts_hold():
    lint_imports = Path(sys.executable).parent / "lint-imports"
    result = subprocess.run([str(lint_imports)], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
