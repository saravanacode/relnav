"""Replay an MCAP recording: frames in order plus the commands recorded with them.

Reads the file as a stream, so a recording cut off mid-write replays up to the cut.
"""

from __future__ import annotations

import struct
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
from mcap.exceptions import McapError
from mcap.records import Channel, Message
from mcap.stream_reader import StreamReader

from relnav.adapters.mcap.codec import (
    TOPIC_COMMANDS,
    TOPIC_DEPTH,
    TOPIC_INFO,
    TOPIC_IR,
    decode_command,
    decode_image,
    decode_info,
    units_to_depth,
)
from relnav.domain.commands import Command
from relnav.domain.types import Frame, Intrinsics


class McapFrameSource:
    def __init__(
        self,
        path: Path,
        *,
        realtime: bool = False,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._file = path.open("rb")
        self._records = iter(StreamReader(self._file).records)
        self._topics: dict[int, str] = {}
        self._intrinsics: Intrinsics | None = None
        self._pending: list[tuple[int, Command]] = []
        self._ready: list[Command] = []
        self._partial: dict[int, dict[str, np.ndarray]] = {}
        self._realtime = realtime
        self._sleep = sleep
        self._clock = clock
        self._start: tuple[int, float] | None = None
        self.truncated = False

    def next_frame(self, timeout_s: float) -> Frame | None:
        while (record := self._next_record()) is not None:
            if isinstance(record, Channel):
                self._topics[record.id] = record.topic
            elif isinstance(record, Message):
                frame = self._handle(record)
                if frame is not None:
                    self._release_commands(frame.timestamp_ns)
                    self._pace(frame.timestamp_ns)
                    return frame
        return None

    def poll(self) -> list[Command]:
        ready, self._ready = self._ready, []
        return ready

    def close(self) -> None:
        self._file.close()

    def _next_record(self) -> object | None:
        # A plain next() rather than a wrapping generator: closing a wrapper that used
        # `yield from` would also close the shared StreamReader iterator.
        try:
            return next(self._records)
        except StopIteration:
            return None
        except (McapError, struct.error):  # a cut inside a record header surfaces as struct.error
            self.truncated = True
            return None

    def _handle(self, message: Message) -> Frame | None:
        topic = self._topics.get(message.channel_id)
        if topic == TOPIC_INFO:
            self._intrinsics = decode_info(message.data)
        elif topic == TOPIC_COMMANDS:
            self._pending.append((message.log_time, decode_command(message.data)))
        elif topic in (TOPIC_IR, TOPIC_DEPTH):
            key = "ir" if topic == TOPIC_IR else "depth"
            index, image = decode_image(message.data, np.uint8 if key == "ir" else np.uint16)
            slot = self._partial.setdefault(index, {})
            slot[key] = image
            if "ir" in slot and "depth" in slot and self._intrinsics is not None:
                del self._partial[index]
                return Frame(
                    index=index,
                    timestamp_ns=message.log_time,
                    ir=slot["ir"],
                    depth_m=units_to_depth(slot["depth"], self._intrinsics.depth_scale),
                    intrinsics=self._intrinsics,
                )
        return None

    def _release_commands(self, timestamp_ns: int) -> None:
        self._ready = [c for t, c in self._pending if t <= timestamp_ns]
        self._pending = [(t, c) for t, c in self._pending if t > timestamp_ns]

    def _pace(self, timestamp_ns: int) -> None:
        if not self._realtime:
            return
        if self._start is None:
            self._start = (timestamp_ns, self._clock())
            return
        first_ns, first_clock = self._start
        delay = first_clock + (timestamp_ns - first_ns) / 1e9 - self._clock()
        if delay > 0:
            self._sleep(delay)
