from typing import Protocol, runtime_checkable

from relnav.domain.types import WorldState


@runtime_checkable
class StateSink(Protocol):
    def publish(self, state: WorldState) -> None: ...

    def close(self) -> None: ...
