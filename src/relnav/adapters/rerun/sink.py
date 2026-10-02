"""Log WorldState to Rerun. Keyframe clouds are logged once in their camera frame;
loop closure only re-logs keyframe transforms, so the map snaps into place cheaply."""

from __future__ import annotations

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from numpy.typing import NDArray

from relnav.adapters.presentation import pin_lines, status_lines
from relnav.domain.types import WorldState

_STOPS = np.array(
    [[40, 90, 220], [40, 200, 220], [80, 200, 80], [240, 220, 60], [230, 70, 50]], dtype=np.float64
)


def height_colors(
    heights_m: NDArray[np.floating], low: float = -0.5, high: float = 2.0
) -> NDArray[np.uint8]:
    t = np.clip((np.asarray(heights_m, dtype=np.float64) - low) / (high - low), 0.0, 1.0)
    x = np.linspace(0.0, 1.0, len(_STOPS))
    channels = [np.interp(t, x, _STOPS[:, c]) for c in range(3)]
    return np.stack(channels, axis=1).astype(np.uint8)


def default_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="World"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/camera/image", name="Infrared"),
                rrb.TextDocumentView(origin="status", name="Status"),
                rrb.TextLogView(origin="events", name="Events"),
                row_shares=[3, 2, 2],
            ),
            column_shares=[3, 2],
        )
    )


def connect(url: str | None = None, spawn: bool = False) -> rr.RecordingStream:
    recording = rr.RecordingStream("relnav")
    if spawn:
        recording.spawn()
    else:
        recording.connect_grpc(url)
    return recording


class RerunSink:
    def __init__(self, recording: rr.RecordingStream) -> None:
        self._rec = recording
        self._pinhole_logged = False
        self._map_version = -1
        self.logged_clouds: set[int] = set()
        rr.log("world", rr.ViewCoordinates.RDF, static=True, recording=recording)
        recording.send_blueprint(default_blueprint())

    def publish(self, state: WorldState) -> None:
        rec, frame = self._rec, state.frame
        rr.set_time("frame", sequence=frame.index, recording=rec)
        if not self._pinhole_logged:
            pinhole = rr.Pinhole(
                image_from_camera=frame.intrinsics.K,
                resolution=[frame.intrinsics.width, frame.intrinsics.height],
            )
            rr.log("world/camera/image", pinhole, static=True, recording=rec)
            self._pinhole_logged = True
        rr.log("world/camera/image/ir", rr.Image(frame.ir), recording=rec)
        if state.T_world_cam is not None:
            T = state.T_world_cam
            rr.log("world/camera", rr.Transform3D(translation=T.t, mat3x3=T.R), recording=rec)
        if state.map_version != self._map_version:
            self._log_keyframes(state)
            self._map_version = state.map_version
        if len(state.path) >= 2:
            path = rr.LineStrips3D([state.path], colors=[(255, 200, 0)], radii=0.01)
            rr.log("world/path", path, recording=rec)
        if state.pins:
            pins = rr.Points3D(
                [p.position_world for p in state.pins],
                labels=[p.label for p in state.pins],
                colors=[(255, 60, 60)],
                radii=0.05,
            )
            rr.log("world/pins", pins, recording=rec)
        status = "\n\n".join(status_lines(state) + pin_lines(state))
        rr.log("status", rr.TextDocument(status), recording=rec)
        for event in state.events:
            rr.log("events", rr.TextLog(event), recording=rec)

    def close(self) -> None:
        self._rec.flush()

    def _log_keyframes(self, state: WorldState) -> None:
        for kf in state.keyframes:
            T = kf.T_world_cam
            rr.log(
                f"world/keyframes/{kf.id}",
                rr.Transform3D(translation=T.t, mat3x3=T.R),
                recording=self._rec,
            )
            if kf.id not in self.logged_clouds and len(kf.cloud_cam):
                heights = -T.apply(kf.cloud_cam)[:, 1]  # world y points down
                cloud = rr.Points3D(kf.cloud_cam, colors=height_colors(heights), radii=0.008)
                rr.log(f"world/keyframes/{kf.id}/cloud", cloud, recording=self._rec)
                self.logged_clouds.add(kf.id)
