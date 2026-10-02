"""Record a walk through the synthetic test room to an MCAP file, so the full pipeline,
HUD and Rerun view can be demoed without the camera.

    uv run python scripts/synthetic_recording.py recordings/synthetic.mcap
    uv run relnav replay recordings/synthetic.mcap --realtime
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))  # make the tests.support package importable

from relnav.adapters.mcap.recorder import McapRecorder  # noqa: E402
from relnav.domain.commands import DropPin  # noqa: E402
from tests.support.synthetic import SyntheticRoom, loop_trajectory  # noqa: E402

# Frames where the "user" presses p. Labels come from config/default.yaml (DOOR, CHAIR, LAPTOP).
PIN_FRAMES = (20, 120, 220)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output", type=Path, help="MCAP file to write")
    parser.add_argument("--frames", type=int, default=300, help="frames in one loop (30 per s)")
    args = parser.parse_args()

    room = SyntheticRoom()
    recorder = McapRecorder(args.output)
    for i, T_world_cam in enumerate(loop_trajectory(args.frames)):
        frame = room.render(T_world_cam, index=i)
        if i in PIN_FRAMES:
            recorder.write_command(frame.timestamp_ns, DropPin())
        recorder.write_frame(frame)
        if i % 50 == 0:
            print(f"rendered {i}/{args.frames}")
    recorder.close()
    print(f"wrote {args.output} ({args.output.stat().st_size / 1e6:.0f} MB)")


if __name__ == "__main__":
    main()
