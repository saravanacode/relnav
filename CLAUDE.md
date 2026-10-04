# relnav — context for Claude Code sessions

relnav ("relative navigation") is a visual SLAM system for an Intel RealSense D435i on a
MacBook Pro M1 Pro (macOS 26). It tracks the camera's 6-DoF pose relative to where it
started and lets the user drop 3D pins on objects, with live distance and bearing to each
pin and pin-to-pin distances. The goal is a 60–90 s LinkedIn demo video plus a reusable,
well-engineered code base.

Current status and to-do list: [docs/STATUS.md](docs/STATUS.md). Read it first.

## Key documents

- Design spec (binding authority): `docs/specs/2026-10-02-relnav-design.md`
- Implementation plan (20 tasks, all built): `docs/plans/2026-10-02-relnav-implementation.md`
- Requirement → test traceability: `docs/requirements.md`
- Field-test procedure (real room, tape measure): `docs/field-test.md`
- Design decisions: `docs/adr/0001…0006`

## Architecture

Ports and adapters, five layers; dependencies point inward only:
`app → adapters → ports → core → domain`. `relnav.core` and `relnav.domain` must never
import adapters, app, pyrealsense2, rerun, mcap or typer (REQ-ARCH-01, enforced by
import-linter in `tests/test_architecture.py` and pre-commit).

- `domain/`: SE3 (`T_a_b` maps frame b → a), types, geometry, commands
- `core/`: ORB features → PnP tracker → keyframes → pose-graph backend (Open3D, loop
  closure) → pins (anchored to keyframes) → health (OK / DEGRADED / LOST) → `SlamPipeline`
- `ports/`: FrameSource, StateSink, CommandSource, Recorder protocols
- `adapters/`: RealSense source, MCAP recorder/replay, HUD (OpenCV), Rerun sink, results
- `app/`: run loop, config (pydantic over YAML), session, verify, Typer CLI

Internal axes follow OpenCV (x right, y down, z forward); world = first tracked frame.
The HUD shows X right, Y up, Z forward.

## Commands (always through uv)

```bash
uv run pytest -m "not hardware"          # 132 tests, ~30 s
uv run ruff check . && uv run ruff format --check .
uv run pyright
uv run lint-imports
uv run relnav viewer                      # Rerun viewer, as the normal user
scripts/relnav-sudo live                  # camera needs root on macOS
scripts/relnav-sudo record recordings/home.mcap
uv run relnav replay recordings/home.mcap --realtime
uv run relnav verify results/home recordings/truth.yaml
uv run python scripts/synthetic_recording.py recordings/synthetic.mcap   # demo without camera
```

Add `--config config/usb2.yaml` (640×480 @ 15 fps) to live/record when the camera is on a
USB 2 link.

## Working rules

- uv for everything (`uv add`, `uv run`); never pip, plain `python`, or `.venv/bin/python`
  in docs or instructions.
- Test-first: write the failing test, watch it fail, then the code. Commit messages end
  with the Co-Authored-By line used in the git history.
- Pre-commit runs ruff, ruff format and the import contracts on every commit.

## Hardware and macOS facts (learned the hard way)

- librealsense needs **root** on macOS. `scripts/relnav-sudo` runs relnav under sudo and
  hands written files back to the user (`SUDO_UID`/`SUDO_GID`).
- Running sudo from Claude Code: no TTY, so use an askpass helper
  (`osascript … display dialog … with hidden answer`) with `sudo -A`; the user types the
  password into a macOS dialog.
- **Color and IMU do not work on macOS**; relnav uses depth + left infrared only. Trying
  the color stream leaves the camera stuck until it is replugged.
- The PyPI `pyrealsense2-macosx` wheel segfaults as root. relnav uses pyrealsense2 2.58.4
  built from source into `vendor/pyrealsense2` (binaries git-ignored; rebuild with
  `scripts/build_pyrealsense2.sh`).
- The Mac's Command Line Tools are broken (clang cannot find `<string>`); the build script
  works around it with `-isystem $SDK/usr/include/c++/v1`.
- Camera firmware is 5.13.0.50 (old; SDK expects 5.17.x). Not updated yet.
- **USB link keeps falling back to USB 2 (480 Mb/s)** because of the cable and plug, not
  the Mac's ports (all three Thunderbolt 4 ports have reached 5 Gb/s). On USB 2 there is
  no 848×480 @ 30 fps mode and power drops to 2.48 W, so streams fail or stop. Check with
  `system_profiler SPUSBHostDataType | grep -A10 RealSense | grep "Link Speed"`.
- The first pipeline start on macOS sometimes fails with `failed to set power state`;
  the source retries 4 times. Replugging clears a stuck camera.
