"""Errors a frame source may raise."""


class FrameTimeout(Exception):
    """No frame arrived in time. Transient: the caller may retry."""


class FrameSourceError(Exception):
    """The frame source cannot continue (device lost, start failed after retries)."""
