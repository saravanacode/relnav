import os

import numpy as np
import pytest

from relnav.adapters import fs
from relnav.adapters.mcap.codec import (
    decode_command,
    decode_image,
    decode_info,
    depth_to_units,
    encode_command,
    encode_image,
    encode_info,
    units_to_depth,
)
from relnav.adapters.mcap.recorder import McapRecorder
from relnav.adapters.mcap.source import McapFrameSource
from relnav.domain.commands import DropPin
from relnav.domain.types import Frame, Intrinsics

INTR = Intrinsics(width=64, height=48, fx=50.0, fy=50.0, cx=32.0, cy=24.0)


def frame(i: int) -> Frame:
    rng = np.random.default_rng(i)
    return Frame(
        index=i,
        timestamp_ns=1_000_000_000 + i * 33_333_333,
        ir=rng.integers(0, 256, (48, 64), dtype=np.uint8),
        depth_m=rng.uniform(0.3, 4.0, (48, 64)).astype(np.float32),
        intrinsics=INTR,
    )


def read_all(source: McapFrameSource):
    frames, commands = [], []
    while (f := source.next_frame(1.0)) is not None:
        frames.append(f)
        commands.append(source.poll())
    source.close()
    return frames, commands


def test_codec_roundtrips():
    image = np.arange(12, dtype=np.uint16).reshape(3, 4)
    index, decoded = decode_image(encode_image(7, image), np.uint16)
    assert index == 7 and np.array_equal(decoded, image)
    assert decode_info(encode_info(INTR)) == INTR
    command = DropPin(label="DOOR", u=10, v=20)
    assert decode_command(encode_command(command)) == command
    depth = np.array([[0.0, 1.2345, 3.9999]], np.float32)
    assert np.allclose(units_to_depth(depth_to_units(depth, 0.001), 0.001), depth, atol=5e-4)


def test_record_then_replay_gives_frames_and_commands(tmp_path):
    path = tmp_path / "run.mcap"
    recorder = McapRecorder(path)
    for i in range(3):
        f = frame(i)
        if i == 1:
            recorder.write_command(f.timestamp_ns, DropPin(label="DOOR"))
        recorder.write_frame(f)
    recorder.close()

    frames, commands = read_all(McapFrameSource(path))
    assert [f.index for f in frames] == [0, 1, 2]
    for original, replayed in zip((frame(i) for i in range(3)), frames, strict=True):
        assert replayed.timestamp_ns == original.timestamp_ns
        assert replayed.intrinsics == INTR
        assert np.array_equal(replayed.ir, original.ir)
        assert np.allclose(replayed.depth_m, original.depth_m, atol=5e-4)
    assert commands == [[], [DropPin(label="DOOR")], []]


def test_truncated_recording_replays_up_to_the_cut(tmp_path):
    """Review focus 1: a recording cut off mid-write still replays, without an exception."""
    path = tmp_path / "cut.mcap"
    recorder = McapRecorder(path, chunk_size=50_000)
    for i in range(40):
        recorder.write_frame(frame(i))
    recorder.close()
    data = path.read_bytes()
    path.write_bytes(data[: int(len(data) * 0.6)])

    source = McapFrameSource(path)
    frames, _ = read_all(source)
    assert 0 < len(frames) < 40
    assert [f.index for f in frames] == list(range(len(frames)))
    assert source.truncated


def test_realtime_paces_by_timestamps(tmp_path):
    path = tmp_path / "paced.mcap"
    recorder = McapRecorder(path)
    for i in range(3):
        recorder.write_frame(frame(i))
    recorder.close()
    now = [0.0]
    sleeps = []

    def sleep(seconds):
        sleeps.append(seconds)
        now[0] += seconds

    read_all(McapFrameSource(path, realtime=True, sleep=sleep, clock=lambda: now[0]))
    assert sleeps == pytest.approx([0.033333333, 0.033333333], abs=1e-6)


def test_files_written_under_sudo_go_back_to_the_user(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setenv("SUDO_UID", "501")
    monkeypatch.setenv("SUDO_GID", "20")
    monkeypatch.setattr(os, "geteuid", lambda: 0)
    monkeypatch.setattr(os, "chown", lambda p, uid, gid: calls.append((str(p), uid, gid)))
    fs.give_to_sudo_user(tmp_path)
    assert calls == [(str(tmp_path), 501, 20)]


def test_files_untouched_without_sudo(tmp_path, monkeypatch):
    monkeypatch.delenv("SUDO_UID", raising=False)
    monkeypatch.setattr(os, "chown", lambda *a: pytest.fail("chown called"))
    fs.give_to_sudo_user(tmp_path)
