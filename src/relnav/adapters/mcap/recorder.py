"""Write frames and commands to an MCAP file (zstd-compressed chunks)."""

from __future__ import annotations

from pathlib import Path

from mcap.writer import CompressionType, Writer

from relnav import __version__
from relnav.adapters.fs import give_to_sudo_user, make_dirs_for_user
from relnav.adapters.mcap.codec import (
    ENCODING,
    SCHEMA_DOC,
    TOPIC_COMMANDS,
    TOPIC_DEPTH,
    TOPIC_INFO,
    TOPIC_IR,
    depth_to_units,
    encode_command,
    encode_image,
    encode_info,
)
from relnav.domain.commands import Command, DropPin
from relnav.domain.types import Frame


class McapRecorder:
    def __init__(self, path: Path, chunk_size: int = 4 << 20) -> None:
        make_dirs_for_user(path.parent)
        self._path = path
        self._file = path.open("wb")
        self._writer = Writer(self._file, chunk_size=chunk_size, compression=CompressionType.ZSTD)
        self._writer.start(profile="relnav", library=f"relnav {__version__}")
        schema_id = self._writer.register_schema(name=ENCODING, encoding=ENCODING, data=SCHEMA_DOC)
        self._channels = {
            topic: self._writer.register_channel(
                topic=topic, message_encoding=ENCODING, schema_id=schema_id
            )
            for topic in (TOPIC_INFO, TOPIC_IR, TOPIC_DEPTH, TOPIC_COMMANDS)
        }
        self._info_written = False
        self._closed = False

    def _add(self, topic: str, timestamp_ns: int, data: bytes) -> None:
        self._writer.add_message(
            channel_id=self._channels[topic],
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=data,
        )

    def write_frame(self, frame: Frame) -> None:
        if not self._info_written:
            self._add(TOPIC_INFO, frame.timestamp_ns, encode_info(frame.intrinsics))
            self._info_written = True
        units = depth_to_units(frame.depth_m, frame.intrinsics.depth_scale)
        self._add(TOPIC_IR, frame.timestamp_ns, encode_image(frame.index, frame.ir))
        self._add(TOPIC_DEPTH, frame.timestamp_ns, encode_image(frame.index, units))

    def write_command(self, timestamp_ns: int, command: Command) -> None:
        if isinstance(command, DropPin):
            self._add(TOPIC_COMMANDS, timestamp_ns, encode_command(command))

    def close(self) -> None:
        if self._closed:
            return
        self._writer.finish()
        self._file.close()
        give_to_sudo_user(self._path)
        self._closed = True
