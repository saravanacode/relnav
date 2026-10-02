from typing import Protocol, runtime_checkable

from relnav.domain.types import Frame


@runtime_checkable
class FrameSource(Protocol):
    def next_frame(self, timeout_s: float) -> Frame | None:
        """Next frame, or None at the end of the stream.

        Raises FrameTimeout (retryable) or FrameSourceError (fatal).
        """
        ...

    def close(self) -> None: ...
