from typing import Protocol, runtime_checkable

from relnav.domain.commands import Command


@runtime_checkable
class CommandSource(Protocol):
    def poll(self) -> list[Command]:
        """Commands that arrived since the last poll. Never blocks."""
        ...
