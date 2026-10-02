from typing import Protocol, runtime_checkable

from relnav.domain.commands import Command
from relnav.domain.types import Frame


@runtime_checkable
class Recorder(Protocol):
    def write_frame(self, frame: Frame) -> None: ...

    def write_command(self, timestamp_ns: int, command: Command) -> None: ...

    def close(self) -> None: ...
