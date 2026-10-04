"""Render the README figures from a replay of the synthetic recording.

    uv run python scripts/synthetic_recording.py recordings/synthetic.mcap
    uv run python scripts/readme_figures.py recordings/synthetic.mcap

Writes docs/images/hud.png (one HUD frame, exactly as the live window draws it) and
docs/images/map.svg (top-down view: estimated path, ground truth, keyframes and pins).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))  # make the tests.support package importable

from relnav.adapters.hud.render import HudRenderer  # noqa: E402
from relnav.adapters.mcap.source import McapFrameSource  # noqa: E402
from relnav.app.config import load_config  # noqa: E402
from relnav.core.pipeline import SlamPipeline  # noqa: E402
from relnav.domain.types import WorldState  # noqa: E402
from tests.support.synthetic import loop_trajectory  # noqa: E402

OUT = ROOT / "docs" / "images"


def replay(recording: Path, hud_frame: int) -> tuple[SlamPipeline, WorldState, np.ndarray, int]:
    cfg = load_config(ROOT / "config" / "default.yaml")
    pipeline = SlamPipeline(cfg.slam)
    source = McapFrameSource(recording)
    renderer = HudRenderer()
    hud = None
    state = None
    frames = 0
    while (frame := source.next_frame(1.0)) is not None:
        state = pipeline.step(frame, source.poll())
        image = renderer.render(state)  # every frame, so the event list matches the live HUD
        if frame.index == hud_frame:
            hud = image
        frames += 1
    source.close()
    if state is None or hud is None:
        raise SystemExit(f"recording has no frame {hud_frame}")
    return pipeline, state, hud, frames


def map_svg(pipeline: SlamPipeline, state: WorldState, frames: int) -> str:
    """Top-down (x right, z up the page) view in meters."""
    est = np.array([T.t for _, T in pipeline.trajectory()])
    truth = np.array([T.t for T in loop_trajectory(frames)])
    keyframes = np.array([kf.T_world_cam.t for kf in state.keyframes])
    pins = [(p.label, np.array(p.position_world)) for p in state.pins]

    xs = np.concatenate([est[:, 0], truth[:, 0], [p[0] for _, p in pins]])
    zs = np.concatenate([est[:, 2], truth[:, 2], [p[2] for _, p in pins]])
    pad = 0.35
    x0, x1, z0, z1 = xs.min() - pad, xs.max() + pad, zs.min() - pad, zs.max() + pad
    width, scale = 760, 760 / (x1 - x0)
    height = int((z1 - z0) * scale)

    def px(p: np.ndarray) -> tuple[float, float]:
        return (p[0] - x0) * scale, (z1 - p[2]) * scale

    def polyline(points: np.ndarray, cls: str) -> str:
        coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in map(px, points))
        return f'<polyline class="{cls}" points="{coords}"/>'

    rmse_cm = 100 * float(np.sqrt(np.mean(np.sum((est - truth[: len(est)]) ** 2, axis=1))))
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height + 70}" '
        f'width="{width}" font-family="ui-monospace, Menlo, monospace" font-size="14">',
        "<style>",
        ".bg{fill:#0f1419}.grid{stroke:#25303a;stroke-width:1}",
        ".truth{fill:none;stroke:#6b7785;stroke-width:6;stroke-linecap:round;opacity:.55}",
        ".est{fill:none;stroke:#3fb6ff;stroke-width:2.5;stroke-linejoin:round}",
        ".kf{fill:#0f1419;stroke:#3fb6ff;stroke-width:1.5}.pin{fill:#ffcc33}",
        ".label{fill:#e6edf3}.muted{fill:#8b98a5}.start{fill:#5ad16a}",
        "</style>",
        f'<rect class="bg" width="{width}" height="{height + 70}" rx="12"/>',
    ]
    for gx in np.arange(np.ceil(x0 * 2) / 2, x1, 0.5):
        x, _ = px(np.array([gx, 0, 0]))
        parts.append(f'<line class="grid" x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="{height}"/>')
    for gz in np.arange(np.ceil(z0 * 2) / 2, z1, 0.5):
        _, y = px(np.array([0, 0, gz]))
        parts.append(f'<line class="grid" x1="0" y1="{y:.1f}" x2="{width}" y2="{y:.1f}"/>')
    parts.append(polyline(truth, "truth"))
    parts.append(polyline(est, "est"))
    for kf in keyframes:
        x, y = px(kf)
        parts.append(f'<circle class="kf" cx="{x:.1f}" cy="{y:.1f}" r="4"/>')
    sx, sy = px(np.zeros(3))
    parts.append(f'<circle class="start" cx="{sx:.1f}" cy="{sy:.1f}" r="7"/>')
    parts.append(f'<text class="label" x="{sx + 12:.1f}" y="{sy + 5:.1f}">start (0, 0, 0)</text>')
    for i, (label, p) in enumerate(pins):
        x, y = px(p)
        dy = -6 + 20 * (i % 2)  # stagger labels of pins that sit close together
        right_half = x > width / 2  # put the label on the inner side so it stays on the card
        anchor, lx = ("end", x - 12) if right_half else ("start", x + 12)
        parts.append(
            f'<path class="pin" d="M{x:.1f},{y:.1f} l-7,-14 a8,8 0 1 1 14,0 z"/>'
            f'<text class="label" text-anchor="{anchor}" x="{lx:.1f}" y="{y + dy:.1f}">'
            f"{label}</text>"
        )
    legend_y = height + 30
    parts += [
        f'<line class="truth" x1="20" y1="{legend_y}" x2="50" y2="{legend_y}"/>',
        f'<text class="muted" x="58" y="{legend_y + 5}">ground truth</text>',
        f'<line class="est" x1="190" y1="{legend_y}" x2="220" y2="{legend_y}"/>',
        f'<text class="muted" x="228" y="{legend_y + 5}">estimated path</text>',
        f'<circle class="kf" cx="380" cy="{legend_y}" r="4"/>',
        f'<text class="muted" x="392" y="{legend_y + 5}">keyframe</text>',
        f'<text class="muted" x="20" y="{legend_y + 28}">{frames} frames · '
        f"{len(keyframes)} keyframes · {pipeline.loop_count} loop closures · "
        f"RMSE {rmse_cm:.1f} cm · grid 0.5 m</text>",
        "</svg>",
    ]
    return "\n".join(parts) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recording", type=Path, help="synthetic MCAP recording")
    parser.add_argument("--hud-frame", type=int, default=240, help="frame to save as hud.png")
    args = parser.parse_args()

    pipeline, state, hud, frames = replay(args.recording, args.hud_frame)
    OUT.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / "hud.png"), hud)
    (OUT / "map.svg").write_text(map_svg(pipeline, state, frames))
    print(f"wrote {OUT / 'hud.png'} and {OUT / 'map.svg'}")


if __name__ == "__main__":
    main()
