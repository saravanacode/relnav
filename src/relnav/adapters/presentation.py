"""Human-readable views of the state, shared by the HUD and the Rerun status panel.

Positions are shown as X right, Y up, Z forward (internally y points down).
"""

from __future__ import annotations

import numpy as np

from relnav.domain.se3 import SE3
from relnav.domain.types import WorldState


def human_position(T_world_cam: SE3) -> tuple[float, float, float]:
    x, y, z = T_world_cam.t
    return float(x), float(-y), float(z)


def yaw_deg(T_world_cam: SE3) -> float:
    forward = T_world_cam.R[:, 2]
    return float(np.degrees(np.arctan2(forward[0], forward[2])))


def format_bearing(azimuth_deg: float) -> str:
    side = "R" if azimuth_deg >= 0 else "L"
    return f"{abs(azimuth_deg):.0f} deg {side}"


def status_lines(state: WorldState) -> list[str]:
    status = state.status
    head = f"STATUS  {status.state}"
    if status.inliers:
        head += f"  ({status.inliers} inliers)"
    lines = [head]
    if status.message:
        lines.append(status.message)
    if state.T_world_cam is None:
        lines.append("POS  --")
    else:
        x, y, z = human_position(state.T_world_cam)
        lines.append(f"POS  X {x:+.2f}  Y {y:+.2f}  Z {z:+.2f} m")
        lines.append(f"YAW  {yaw_deg(state.T_world_cam):+.0f} deg")
    return lines


def pin_lines(state: WorldState) -> list[str]:
    lines = [
        f"{pin.label:<8}  {pin.distance_m:.2f} m  {format_bearing(pin.azimuth_deg)}"
        for pin in state.pins
    ]
    lines += [f"{p.label_a}-{p.label_b}  {p.distance_m:.2f} m" for p in state.pin_pairs]
    return lines


def fps(state: WorldState) -> float | None:
    total = state.timings_ms.get("total_ms")
    return 1000.0 / total if total else None
