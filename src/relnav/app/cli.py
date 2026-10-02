"""relnav command line."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from relnav.app.config import AppConfig, load_config
from relnav.app.runner import RunSummary
from relnav.app.session import run_session
from relnav.app.verify import check_pairs, load_pins, load_truth
from relnav.ports.errors import FrameSourceError
from relnav.ports.frame_source import FrameSource

app = typer.Typer(
    help="relnav: relative navigation with an Intel RealSense D435i.",
    no_args_is_help=True,
    add_completion=False,
)
console = Console()
DEFAULT_CONFIG = Path("config/default.yaml")

ConfigOption = Annotated[
    Path | None, typer.Option("--config", "-c", help="YAML config (default: config/default.yaml)")
]
ResultsOption = Annotated[Path, typer.Option(help="Folder for run results")]
NameOption = Annotated[str | None, typer.Option(help="Results subfolder name")]
HudOption = Annotated[bool, typer.Option("--hud/--no-hud", help="Show the operator HUD window")]
RerunOption = Annotated[
    bool, typer.Option("--rerun/--no-rerun", help="Stream to the Rerun 3D viewer")
]


def _config(path: Path | None) -> AppConfig:
    if path is None and DEFAULT_CONFIG.exists():
        path = DEFAULT_CONFIG
    return load_config(path)


def _open_camera(cfg: AppConfig) -> FrameSource:
    """Start the camera, turning a start failure into a message and exit code 1."""
    from relnav.adapters.realsense.source import RealSenseFrameSource

    try:
        return RealSenseFrameSource(cfg.camera)
    except FrameSourceError as exc:
        console.print(f"[red]camera error:[/] {exc}")
        console.print("Unplug and replug the camera, check it shows 5 Gb/s, then retry.")
        raise typer.Exit(1) from None


def _report(summary: RunSummary) -> None:
    console.print(
        f"[bold]{summary.frames}[/] frames, {summary.timeouts} timeouts, "
        f"stopped: {summary.stop_reason}"
    )


@app.command()
def viewer() -> None:
    """Open the Rerun viewer. Run this as your normal user, not with sudo."""
    raise typer.Exit(subprocess.call([sys.executable, "-m", "rerun"]))


@app.command()
def live(
    config: ConfigOption = None,
    record: Annotated[Path | None, typer.Option(help="Also record to this MCAP file")] = None,
    results: ResultsOption = Path("results"),
    name: NameOption = None,
    hud: HudOption = True,
    rerun: RerunOption = True,
) -> None:
    """Track live from the camera (needs root: scripts/relnav-sudo live)."""
    from relnav.adapters.mcap.recorder import McapRecorder

    cfg = _config(config)
    source = _open_camera(cfg)
    summary = run_session(
        source,
        cfg=cfg,
        with_slam=True,
        hud=hud,
        rerun=rerun,
        recorder=McapRecorder(record) if record else None,
        results_dir=results / (name or time.strftime("live-%Y%m%d-%H%M%S")),
    )
    _report(summary)


@app.command()
def record(
    output: Annotated[Path, typer.Argument(help="MCAP file to write")],
    config: ConfigOption = None,
    hud: HudOption = True,
) -> None:
    """Record camera frames and pin presses (needs root: scripts/relnav-sudo record x.mcap)."""
    from relnav.adapters.mcap.recorder import McapRecorder

    cfg = _config(config)
    source = _open_camera(cfg)
    summary = run_session(
        source, cfg=cfg, with_slam=False, hud=hud, rerun=False, recorder=McapRecorder(output)
    )
    _report(summary)


@app.command()
def replay(
    recording: Annotated[Path, typer.Argument(help="MCAP recording to replay")],
    config: ConfigOption = None,
    realtime: Annotated[bool, typer.Option(help="Pace to the recorded timestamps")] = False,
    results: ResultsOption = Path("results"),
    name: NameOption = None,
    hud: HudOption = True,
    rerun: RerunOption = True,
) -> None:
    """Run the full pipeline on a recording. No camera or root needed."""
    from relnav.adapters.mcap.source import McapFrameSource

    cfg = _config(config)
    source = McapFrameSource(recording, realtime=realtime)
    summary = run_session(
        source,
        cfg=cfg,
        with_slam=True,
        hud=hud,
        rerun=rerun,
        extra_command_sources=[source],
        results_dir=results / (name or recording.stem),
    )
    if source.truncated:
        console.print("[yellow]warning:[/] the recording was cut off; replayed up to the cut")
    _report(summary)


@app.command()
def verify(
    results_dir: Annotated[Path, typer.Argument(help="Run folder containing pins.json")],
    truth: Annotated[Path, typer.Argument(help="YAML with tape-measured pin distances")],
    tolerance_cm: Annotated[float, typer.Option(help="Pass threshold (REQ-ACC-01)")] = 3.0,
) -> None:
    """Compare measured pin-to-pin distances with tape measurements."""
    try:
        checks = check_pairs(load_pins(results_dir), load_truth(truth))
    except (OSError, ValueError, KeyError) as exc:
        console.print(f"[red]error:[/] {exc}")
        raise typer.Exit(2) from None
    table = Table(title=f"REQ-ACC-01: pin-to-pin distance, tolerance {tolerance_cm:g} cm")
    for column in ("Pair", "Measured (m)", "Truth (m)", "Error (cm)", "Result"):
        table.add_column(column)
    all_passed = True
    for check in checks:
        passed = check.passed(tolerance_cm)
        all_passed &= passed
        if check.measured_m is None or check.error_cm is None:
            row = ("--", f"{check.truth_m:.3f}", "--", "[red]MISSING[/]")
        else:
            verdict = "[green]PASS[/]" if passed else "[red]FAIL[/]"
            row = (
                f"{check.measured_m:.3f}",
                f"{check.truth_m:.3f}",
                f"{check.error_cm:+.1f}",
                verdict,
            )
        table.add_row(f"{check.a} - {check.b}", *row)
    console.print(table)
    raise typer.Exit(0 if all_passed else 1)
