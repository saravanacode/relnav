"""Camera settings. Kept free of pyrealsense2 so config loads without the bindings."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RealSenseConfig:
    width: int = 848
    height: int = 480
    fps: int = 30
    # Off: the projector's dot pattern moves with the camera and confuses tracking.
    emitter_enabled: bool = False
    min_depth_m: float = 0.3
    max_depth_m: float = 4.0
    start_attempts: int = 4
    start_retry_delay_s: float = 2.0
    queue_size: int = 2
    max_missed_waits: int = 5
