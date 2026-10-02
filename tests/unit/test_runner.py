import numpy as np

from relnav.app.runner import run
from relnav.domain.commands import DropPin, Quit
from relnav.domain.types import Frame, Intrinsics, WorldState
from relnav.ports.errors import FrameSourceError, FrameTimeout

INTR = Intrinsics(4, 3, 2.0, 2.0, 2.0, 1.5)


def frame(i: int) -> Frame:
    return Frame(i, i * 1000, np.zeros((3, 4), np.uint8), np.zeros((3, 4), np.float32), INTR)


class FakeSource:
    def __init__(self, items):
        self.items = list(items)
        self.closed = False

    def next_frame(self, timeout_s):
        if not self.items:
            return None
        item = self.items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.closed = True


class ListSink:
    def __init__(self):
        self.states: list[WorldState] = []
        self.closed = False

    def publish(self, state):
        self.states.append(state)

    def close(self):
        self.closed = True


class ScriptedCommands:
    def __init__(self, batches):
        self.batches = list(batches)

    def poll(self):
        return self.batches.pop(0) if self.batches else []


class MemoryRecorder:
    def __init__(self):
        self.log = []
        self.closed = False

    def write_frame(self, frame):
        self.log.append(("frame", frame.index))

    def write_command(self, timestamp_ns, command):
        self.log.append(("command", timestamp_ns, command))

    def close(self):
        self.closed = True


class EchoPipeline:
    def __init__(self):
        self.calls = []

    def step(self, frame, commands):
        self.calls.append((frame.index, list(commands)))
        return WorldState.preview(frame, "echo")


def test_runs_until_end_of_stream():
    source, sink = FakeSource([frame(0), frame(1), frame(2)]), ListSink()
    summary = run(source, [sink])
    assert (summary.frames, summary.stop_reason) == (3, "end of stream")
    assert [s.frame.index for s in sink.states] == [0, 1, 2]
    assert sink.states[0].status.message == "recording"
    assert source.closed and sink.closed


def test_timeouts_are_retried():
    """REQ-REL-01: a frame timeout never crashes the run."""
    summary = run(FakeSource([FrameTimeout(), FrameTimeout(), frame(0)]), [ListSink()])
    assert (summary.frames, summary.timeouts, summary.stop_reason) == (1, 2, "end of stream")


def test_too_many_timeouts_stop_the_run():
    sink = ListSink()
    summary = run(FakeSource([FrameTimeout()] * 5), [sink], max_consecutive_timeouts=5)
    assert summary.stop_reason == "no frames for 5 attempts"
    assert summary.frames == 0 and sink.closed


def test_source_error_stops_cleanly():
    """REQ-REL-01 / review focus 4: camera unplugged mid-run."""
    source, sink, recorder = (
        FakeSource([frame(0), FrameSourceError("device lost")]),
        ListSink(),
        MemoryRecorder(),
    )
    summary = run(source, [sink], recorder=recorder)
    assert summary.stop_reason == "source error: device lost"
    assert summary.frames == 1
    assert sink.closed and recorder.closed and source.closed


def test_quit_command_stops_before_the_frame():
    commands = ScriptedCommands([[], [Quit()]])
    summary = run(
        FakeSource([frame(0), frame(1), frame(2)]), [ListSink()], command_sources=[commands]
    )
    assert (summary.frames, summary.stop_reason) == (1, "quit")


def test_commands_are_recorded_before_their_frame():
    recorder = MemoryRecorder()
    commands = ScriptedCommands([[DropPin(label="DOOR")]])
    run(FakeSource([frame(0)]), [ListSink()], command_sources=[commands], recorder=recorder)
    assert recorder.log == [("command", 0, DropPin(label="DOOR")), ("frame", 0)]


def test_pipeline_gets_the_frame_commands():
    pipeline = EchoPipeline()
    commands = ScriptedCommands([[DropPin()], []])
    run(
        FakeSource([frame(0), frame(1)]),
        [ListSink()],
        pipeline=pipeline,
        command_sources=[commands],
    )
    assert pipeline.calls == [(0, [DropPin()]), (1, [])]


def test_record_mode_marks_pins_for_replay():
    sink = ListSink()
    run(FakeSource([frame(0)]), [sink], command_sources=[ScriptedCommands([[DropPin()]])])
    assert sink.states[0].events == ("pin marked (applied during replay)",)
