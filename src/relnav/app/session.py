"""Assemble sinks and command sources, run, then write results."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import structlog

from relnav.adapters.results.writer import TimingCollector, write_results
from relnav.app.config import AppConfig
from relnav.app.runner import RunSummary, run
from relnav.core.pipeline import SlamPipeline
from relnav.domain.types import WorldState
from relnav.ports.commands import CommandSource
from relnav.ports.frame_source import FrameSource
from relnav.ports.recorder import Recorder
from relnav.ports.state_sink import StateSink

log = structlog.get_logger("relnav")


class EventLog:
    """StateSink that writes pipeline events (pins, loops, lost/relocalized) to the log."""

    def publish(self, state: WorldState) -> None:
        for event in state.events:
            log.info(event, frame=state.frame.index, status=str(state.status.state))

    def close(self) -> None:
        pass


def run_session(
    source: FrameSource,
    *,
    cfg: AppConfig,
    with_slam: bool,
    hud: bool,
    rerun: bool,
    recorder: Recorder | None = None,
    extra_command_sources: Sequence[CommandSource] = (),
    results_dir: Path | None = None,
) -> RunSummary:
    sinks: list[StateSink] = [EventLog()]
    command_sources: list[CommandSource] = list(extra_command_sources)
    if hud:
        from relnav.adapters.hud.window import HudWindow

        window = HudWindow()
        sinks.append(window)
        command_sources.append(window)
    if rerun:
        from relnav.adapters.rerun.sink import RerunSink, connect

        sinks.append(RerunSink(connect(cfg.rerun_url, cfg.rerun_spawn)))
    pipeline = SlamPipeline(cfg.slam) if with_slam else None
    timing = TimingCollector()
    if pipeline is not None:
        sinks.append(timing)

    summary = run(
        source, sinks, pipeline=pipeline, command_sources=command_sources, recorder=recorder
    )
    log.info("run finished", frames=summary.frames, reason=summary.stop_reason)

    if pipeline is not None and results_dir is not None:
        pins = [
            (pin.label, pipeline.pins.world_position(pin, pipeline.keyframes))
            for pin in pipeline.pins.pins
        ]
        write_results(
            results_dir,
            trajectory=pipeline.trajectory(),
            pins=pins,
            timing=timing.summary(),
            summary={
                "frames": summary.frames,
                "timeouts": summary.timeouts,
                "dropped_frames": getattr(source, "dropped_frames", 0),
                "stop_reason": summary.stop_reason,
                "keyframes": len(pipeline.keyframes),
                "loop_closures": pipeline.loop_count,
            },
        )
        log.info("results written", path=str(results_dir))
    return summary
