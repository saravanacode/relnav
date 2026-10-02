"""User commands. They carry no timing: the runner applies them to the current frame."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DropPin:
    """Drop a pin at pixel (u, v); None means the image center, where the crosshair is."""

    label: str | None = None
    u: int | None = None
    v: int | None = None


@dataclass(frozen=True)
class Quit:
    """Stop the run."""


Command = DropPin | Quit
