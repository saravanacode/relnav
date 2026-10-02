"""The run loop shared by live, record and replay: source -> (recorder) -> pipeline -> sinks."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

from relnav.domain.commands import Command, DropPin, Quit
from relnav.domain.types import Frame, WorldState
from relnav.ports.commands import CommandSource
from relnav.ports.errors import FrameSourceError, FrameTimeout
from relnav.ports.frame_source import FrameSource
from relnav.ports.recorder import Recorder
from relnav.ports.state_sink import StateSink


class Pipeline(Protocol):
    def step(self, frame: Frame, commands: Sequence[Command]) -> WorldState: ...


@dataclass(frozen=True)
class RunSummary:
    frames: int
    timeouts: int
    stop_reason: str


def run(
    source: FrameSource,
    sinks: Sequence[StateSink],
    *,
    pipeline: Pipeline | None = None,
    command_sources: Sequence[CommandSource] = (),
    recorder: Recorder | None = None,
    timeout_s: float = 2.0,
    max_consecutive_timeouts: int = 5,
) -> RunSummary:
    frames = timeouts = consecutive_timeouts = 0
    reason = "end of stream"
    try:
        while True:
            try:
                frame = source.next_frame(timeout_s)
            except FrameTimeout:
                timeouts += 1
                consecutive_timeouts += 1
                if consecutive_timeouts >= max_consecutive_timeouts:
                    reason = f"no frames for {consecutive_timeouts} attempts"
                    break
                continue
            except FrameSourceError as exc:
                reason = f"source error: {exc}"
                break
            if frame is None:
                break
            consecutive_timeouts = 0

            commands = [c for s in command_sources for c in s.poll()]
            if any(isinstance(c, Quit) for c in commands):
                reason = "quit"
                break
            if recorder is not None:
                for command in commands:
                    recorder.write_command(frame.timestamp_ns, command)
                recorder.write_frame(frame)

            if pipeline is not None:
                state = pipeline.step(frame, commands)
            else:
                marks = tuple(
                    "pin marked (applied during replay)" for c in commands if isinstance(c, DropPin)
                )
                state = WorldState.preview(frame, "recording", marks)
            for sink in sinks:
                sink.publish(state)
            frames += 1
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception as exc:  # report it and still close everything and write results
        reason = f"error: {type(exc).__name__}: {exc}"
    finally:
        closers = [*([recorder.close] if recorder is not None else []), *(s.close for s in sinks)]
        failures = [_close(fn) for fn in [*closers, source.close]]
        failures = [f for f in failures if f]
        if failures:
            reason += "; close failed: " + ", ".join(failures)
    return RunSummary(frames, timeouts, reason)


def _close(close: Callable[[], None]) -> str:
    """Close one resource; return its error text so one failure cannot skip the rest."""
    try:
        close()
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"
    return ""
