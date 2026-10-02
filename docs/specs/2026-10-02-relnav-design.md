# relnav — Relative Navigation with an Intel RealSense D435i

**Date:** 2026-10-02
**Status:** Draft for review
**Author:** Saravana Perumal (with Claude Code)

## 1. Purpose

relnav is a visual SLAM system that answers two questions in real time:

1. **Where is the camera?** Its 6-DoF pose relative to where it started.
2. **Where are things?** The 3D coordinates of user-marked points ("pins"), and their distance and bearing relative to the camera and to each other.

The first deliverable is a 60–90 second LinkedIn video demo recorded in a real home. The code base is also meant to be a reusable foundation for later projects (other sensors, ROS 2, embedded boards), so it is built with a ports-and-adapters architecture, requirement IDs, and hardware-free tests.

## 2. Success criteria

The project is done when all of the following hold:

1. A recorded walk through a real room replays through the full pipeline and produces a trajectory, pins, and the HUD/3D views needed for the demo video.
2. Measured pin-to-pin distances match tape-measured distances within the accuracy requirements in Section 4.
3. The core (domain + core packages) has no import of camera, visualization, file-format, or CLI code. This is enforced by an automated check.
4. Unit and synthetic tests run without the camera and pass.
5. Replaying the same recording twice produces identical outputs.

## 3. Scope

### In scope

- Live tracking from the RealSense D435i (depth + left infrared) on macOS.
- Recording raw sensor data and user commands to MCAP; deterministic replay.
- Feature-based visual odometry (ORB + PnP RANSAC) with keyframes.
- Loop closure by proximity search plus geometric verification; pose-graph optimization.
- Pins anchored to keyframes, with relative distance and bearing queries.
- Tracking health monitoring (OK / DEGRADED / LOST) and relocalization against keyframes.
- HUD (OpenCV window) and 3D view (Rerun).
- `verify` command that compares measured distances with ground truth.

### Out of scope (for now)

IMU fusion, color camera, ROS 2 integration, dense meshing (TSDF), GPU acceleration, web UI, bag-of-words place recognition, multi-session maps. The port interfaces leave room for these.

### Platform constraints (observed on this machine)

- MacBook with Apple M1 Pro, macOS 26, Python 3.12.
- D435i firmware 5.13.0.50. Depth and infrared streams work at 848×480 @ 30 fps. Color and IMU do not work on macOS.
- librealsense needs root on macOS, so live capture and recording run under `sudo`.
- The PyPI `pyrealsense2-macosx` wheel crashes when opening the device as root. relnav uses `pyrealsense2` 2.58.4 built from source.
- The first pipeline start sometimes fails with `failed to set power state`; a retry succeeds.

## 4. Requirements

Each requirement has an ID and a verification method. `docs/requirements.md` will carry the same table plus links to the tests.

| ID | Requirement | Verified by |
|---|---|---|
| REQ-ACC-01 | Pin-to-pin distance error < 3 cm for pins 1–3 m from the camera when marked. | `relnav verify` against tape measurements |
| REQ-ACC-02 | After a closed loop of about 10 m, a re-observed pin's position differs from its original position by < 5 cm. | Replay of loop recording, pin re-observation check |
| REQ-ACC-03 | On the synthetic test sequence, absolute trajectory error (RMSE) < 2 cm. | Synthetic test |
| REQ-PERF-01 | Live processing mean ≥ 25 fps on the M1 Pro at 848×480. | Timing log in replay benchmark |
| REQ-REL-01 | A frame timeout or failed pipeline start never crashes the app; it retries and reports status. | Unit test with fake frame source |
| REQ-REL-02 | When tracking is lost, the system reports LOST and recovers to OK when the camera returns to a mapped area. | Synthetic test with a gap in the sequence |
| REQ-REP-01 | Replaying the same MCAP file twice produces identical trajectory and pin outputs. | Replay determinism test |
| REQ-ARCH-01 | `relnav.domain` and `relnav.core` do not import `relnav.adapters`, `relnav.app`, `pyrealsense2`, `rerun`, `mcap`, or `typer`. | import-linter contract in CI |

## 5. Architecture

### 5.1 Layers and dependency rule

```
app        → wires everything, CLI
adapters   → implement ports with real libraries (RealSense, MCAP, Rerun, OpenCV GUI)
ports      → interfaces (typing.Protocol)
core       → algorithms: tracking, mapping, pins, health
domain     → plain data types and math
```

Dependencies point inward only: `app → adapters → ports → core → domain`. The core may use computation libraries (NumPy, OpenCV, Open3D) but no I/O, hardware, or UI library.

### 5.2 Package layout

```
relnav/
  pyproject.toml, uv.lock, .python-version (3.12)
  config/default.yaml
  src/relnav/
    domain/
      se3.py          SE3 pose type: compose, inverse, apply to points, from/to (R, t)
      types.py        Intrinsics, Frame, Keyframe, Pin, PinObservation, TrackingStatus, WorldState
      geometry.py     deprojection, depth patch median, bearing/elevation
    core/
      features.py     ORB detection with grid distribution, descriptor matching
      tracker.py      frame-to-keyframe visual odometry, motion model
      keyframes.py    keyframe selection policy and keyframe store
      backend.py      pose graph, loop candidate search, loop verification, optimization
      pins.py         PinRegistry: add pin, world positions, relative queries
      health.py       HealthMonitor state machine
      pipeline.py     SlamPipeline: per-frame step that ties the above together
    ports/
      frame_source.py FrameSource protocol
      state_sink.py   StateSink protocol
      commands.py     CommandSource protocol, Command types
      recorder.py     Recorder protocol
    adapters/
      realsense/      RealSenseFrameSource (live)
      mcap/           McapRecorder, McapFrameSource (replay)
      rerun/          RerunSink (3D view)
      hud/            HudWindow (OpenCV window: StateSink + CommandSource)
      results/        ResultsWriter (TUM trajectory + pins.json)
    app/
      cli.py          Typer app: live, record, replay, verify, viewer
      config.py       Pydantic settings model loaded from YAML
      runner.py       Main loop for live/replay
  tests/
    unit/  synthetic/  replay/
  docs/
    specs/  requirements.md  adr/
  scripts/build_pyrealsense2.sh
  vendor/pyrealsense2/   locally built bindings (binaries git-ignored)
```

## 6. Domain model

- **SE3**: rigid transform stored as a 4×4 float64 matrix. Naming convention `T_a_b` maps points from frame b into frame a. `T_world_cam` is the camera pose.
- **Intrinsics**: `width, height, fx, fy, cx, cy, depth_scale`. The D435i depth stream has no lens distortion model to apply.
- **Frame**: `index, timestamp_ns, ir (H×W uint8), depth_m (H×W float32, 0 = no data), intrinsics`.
- **Keyframe**: `id, frame_index, T_world_cam, keypoints (N×2), descriptors (N×32 uint8), points_cam (N×3, camera frame), cloud_cam (sparse display points)`.
- **Pin**: `id, label, anchor_keyframe_id, point_in_keyframe (3,)`. World position = `T_world_kf · point_in_keyframe`, so pins move with the map when loop closure corrects keyframe poses.
- **TrackingStatus**: `INITIALIZING | OK | DEGRADED | LOST`, plus `inliers`, `reprojection_rmse_px`.
- **WorldState**: snapshot sent to sinks after each frame: frame, `T_world_cam`, status, keyframe poses, path, pins with world positions, relative pin readouts, and per-frame timing.

### Coordinate conventions

- Internally all frames use the OpenCV camera convention: x right, y down, z forward.
- The world frame is the camera frame of the first successfully tracked frame.
- Rerun logs the world with `RDF` view coordinates, so no conversion is needed there.
- The HUD shows positions in a human convention: X right, Y up, Z forward (Y negated).
- **Bearing** to a pin, in the current camera frame: azimuth = `atan2(x, z)` (positive = right), elevation = `atan2(-y, hypot(x, z))` (positive = up).

## 7. Ports

```python
class FrameSource(Protocol):
    intrinsics: Intrinsics
    def next_frame(self, timeout_s: float) -> Frame | None   # None = end of stream
    def close(self) -> None

class StateSink(Protocol):
    def publish(self, state: WorldState) -> None
    def close(self) -> None

class CommandSource(Protocol):
    def poll(self) -> list[Command]                          # non-blocking

class Recorder(Protocol):
    def write_frame(self, frame: Frame) -> None
    def write_command(self, timestamp_ns: int, command: Command) -> None
    def close(self) -> None
```

**Commands:** `DropPin(label: str | None)`, `Quit()`. Commands carry the timestamp of the frame they apply to, so replay applies them on the same frame.

**Errors:**
- `FrameTimeout`: transient; the runner retries.
- `FrameSourceError`: device lost or start failed after retries; the runner stops cleanly.

## 8. Core components

### 8.1 Features (`core/features.py`)

- ORB with 1000 features. Each image is split into an 8×6 grid and the strongest features per cell are kept, so features spread across the image.
- Only keep features whose depth is valid (inside 0.3–4.0 m).
- Matching: brute-force Hamming distance, Lowe ratio test at 0.8.

### 8.2 Tracker (`core/tracker.py`)

For each frame:

1. Predict the pose with a constant-velocity motion model.
2. Match the current frame's features to the reference keyframe's features. The keyframe features have 3D points in the world frame.
3. Solve `cv2.solvePnPRansac` (3D world points ↔ 2D pixels, SQPNP, reprojection threshold 3 px, 100 iterations). Use the predicted pose as the initial guess.
4. Refine on the inliers with `cv2.solvePnPRefineLM`.
5. Output `T_world_cam`, inlier count, and reprojection RMSE.

**Failure:** fewer than 20 inliers, or PnP fails. The tracker then reports failure and the pose is held.

### 8.3 Keyframes (`core/keyframes.py`)

A new keyframe is created when any of these is true:
- translation from the reference keyframe > 0.15 m
- rotation from the reference keyframe > 15°
- inliers < 50% of the reference keyframe's feature count
- inliers < 100

The new keyframe becomes the tracking reference.

### 8.4 Backend (`core/backend.py`)

- **Pose graph:** one node per keyframe. One odometry edge between consecutive keyframes (relative transform from tracking), with information weighted by inlier count.
- **Loop candidates:** checked on each new keyframe. Candidates are keyframes within 1.0 m and 45° viewing direction of the new keyframe, excluding the 10 most recent keyframes.
- **Loop verification:** ORB matching plus PnP RANSAC between the new keyframe and the candidate. The loop is accepted with ≥ 60 inliers and reprojection RMSE ≤ 2 px. Accepted loops add a loop edge.
- **Optimization:** Open3D `global_optimization` with Levenberg–Marquardt, reference node 0, edge prune threshold 0.25. It runs synchronously after a loop edge is added. Keyframe poses are then updated, and the current pose is re-expressed relative to its reference keyframe.

**Assumption:** proximity-based candidates are adequate because room-scale drift before closure is well under 1 m. This is stated as a risk in Section 13.

### 8.5 Pins (`core/pins.py`)

- **Drop pin:** take the median of valid depth in a 9×9 patch at the crosshair. At least 30% of the patch must be valid, otherwise the drop is rejected with a reason.
- **Anchoring:** the 3D point is stored in the current reference keyframe's frame.
- **Labels:** taken in order from `config.pins.labels` (for example `[DOOR, CHAIR, LAPTOP]`), falling back to `P1, P2, …`.
- **Queries:** pin world position, camera→pin distance and bearing, pin↔pin distance for every pair.

### 8.6 Health (`core/health.py`)

State machine:

- `INITIALIZING → OK` on the first frame with ≥ 100 valid features. This frame becomes the world origin and the first keyframe.
- `OK ↔ DEGRADED`: DEGRADED when inliers < 60 or RMSE > 2 px; back to OK when both recover.
- `→ LOST` after 3 consecutive tracking failures.
- While LOST, each frame tries **relocalization**: PnP against the 10 nearest keyframes to the last known pose, then all keyframes. Success (≥ 60 inliers) → OK.

### 8.7 Pipeline (`core/pipeline.py`)

`SlamPipeline.step(frame, commands) -> WorldState` is a pure function of its inputs plus internal state. There is no I/O and no wall-clock time inside the core.

## 9. Adapters

- **RealSenseFrameSource:**
  - Streams: depth Z16 and infrared 1 Y8 at 848×480 @ 30 fps.
  - Emitter off by default (config option), because the projector dot pattern moves with the camera and corrupts visual odometry features.
  - Depth filters: threshold 0.3–4.0 m and spatial. No temporal filter, because it smears depth during motion.
  - Pipeline start is retried up to 4 times with 2 s delays.
  - A capture thread feeds a bounded queue of size 2 that drops the oldest frame and counts drops.
- **McapRecorder / McapFrameSource:**
  - Topics: `/camera/info` (intrinsics, written once), `/camera/ir`, `/camera/depth`, `/commands`.
  - Custom versioned message encoding `relnav.v1`: a small binary header plus raw pixel bytes.
  - zstd chunk compression.
  - Expected size: about 1–1.5 GB for a 2-minute walk.
- **RerunSink:** connects to a Rerun viewer over gRPC. It logs:
  - the camera transform and pinhole model
  - the IR image
  - keyframe point clouds, colored by height
  - the path as a line strip
  - pins as labeled points
  - status text and timings

  A blueprint fixes the layout for recording the video.
- **HudWindow:** OpenCV window showing the IR feed, crosshair, status badge, pose readout, per-pin distance and bearing, and pin↔pin distances. Keys: `p` drop pin, `q` quit. It implements both StateSink and CommandSource.
- **ResultsWriter:** at the end of a run, writes `trajectory.tum` (TUM format, compatible with the `evo` evaluation tool), `pins.json`, and `timing.json` to `results/<run-name>/`.

## 10. Command line (Typer)

| Command | Root? | Purpose |
|---|---|---|
| `relnav viewer` | no | Launch the Rerun viewer as the normal user |
| `relnav live` | yes | Live tracking with HUD and Rerun, optional `--record out.mcap` |
| `relnav record out.mcap` | yes | Record frames and commands; HUD shows the camera feed and crosshair only (no tracking). Pressing `p` records a `DropPin` command, which is applied during replay on the same frame |
| `relnav replay in.mcap` | no | Run the full pipeline on a recording; `--realtime` paces to recorded timestamps, otherwise runs as fast as possible |
| `relnav verify results/<run> truth.yaml` | no | Compare pin-to-pin distances with tape measurements and print an error table |

`truth.yaml` format:

```yaml
pairs:
  - {a: DOOR, b: CHAIR, distance_m: 2.43}
```

The verify table shows, for each pair: measured, truth, error (cm), and pass/fail against REQ-ACC-01.

### Runtime topology

Live and record run under `sudo` in one process. The Rerun viewer runs separately as the normal user, started with `relnav viewer`, and the app connects to it over gRPC. Any files written as root are chowned back to `SUDO_UID:SUDO_GID`.

## 11. Data flow and timing

```
FrameSource ──► SlamPipeline.step ──► WorldState ──► sinks (HUD, Rerun, results)
                       ▲
CommandSource ─────────┘          (Recorder taps frames and commands before step)
```

- **Live:** if processing falls behind, the capture queue drops frames and the HUD shows the drop count.
- **Replay:** every frame is processed in order, with no drops. This is what makes it deterministic.
- **Timing:** each step reports feature extraction, matching, PnP, and backend times, in milliseconds.

## 12. Testing strategy

| Level | What | Hardware |
|---|---|---|
| Unit | SE3 algebra (Hypothesis property tests: inverse, composition, round trips), geometry, pin math, health state machine, keyframe policy, MCAP encode/decode round trip | none |
| Architecture | import-linter contract for REQ-ARCH-01 | none |
| Synthetic | Textured room rendered with Open3D raycasting along a scripted trajectory, including a loop and a "look away" gap. Produces IR-like intensity and depth. Checks REQ-ACC-03, REQ-REL-02, and loop closure. | none |
| Replay | Short recorded MCAP clip (kept out of git, path from env var) → determinism check (REQ-REP-01) and golden-trajectory regression | recording only |
| Field | Real room walk + tape measurements → `relnav verify` (REQ-ACC-01/02) | camera |

**Determinism:** `cv2.setRNGSeed(0)` is called at pipeline start, OpenCV runs single-threaded during replay, and there's no wall clock in the core.

**Synthetic texture:** room surfaces get random-noise textures, so ORB has features to track.

## 13. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Textureless surfaces (plain walls, doors) with the emitter off | Few features, or no depth at the pin | Feature grid; HUD shows "no depth" so the user aims at the frame or handle; emitter interleave mode as a later option |
| Proximity loop search misses loops if drift > 1 m | No correction | Room-scale demo keeps drift small; bag-of-words is a later option |
| Camera USB drops to 2.0 or disconnects | Lost frames | Retry logic; documented plug-in procedure; record-then-replay workflow |
| Processing below 25 fps | Dropped frames live | Replay mode for the demo; features capped at 1000 |
| macOS GUI windows under sudo | HUD may not show | Verified early; fallback is Rerun-only display |

## 14. Tooling and dependencies

- **Runtime:** numpy, opencv-python 5, open3d 0.20, rerun-sdk 0.38, mcap (+ zstandard), typer, pydantic 2, pyyaml, structlog. pyrealsense2 2.58.4 comes from `vendor/` as an editable local package.
- **Dev:** pytest, hypothesis, ruff (lint + format), pyright (type check), import-linter, pre-commit.
- **Package management:** uv for everything (`uv add`, `uv run`).

## 15. Documentation deliverables

- `README.md`: what it is, the demo GIF, the architecture diagram, quick start.
- `docs/requirements.md`: the requirement table with test links.
- `docs/adr/`:
  - `0001-ports-and-adapters`
  - `0002-feature-based-vo`
  - `0003-mcap-recordings`
  - `0004-rerun-visualization`
  - `0005-emitter-off`
  - `0006-pins-anchored-to-keyframes`

## 16. Demo plan (summary)

1. Record a slow walk through the room with `relnav record`, dropping pins on a door, chair, and laptop.
2. Replay with `relnav replay --realtime`, with the HUD and Rerun side by side, and screen-record with macOS.
3. Film a tape-measure check with a phone.
4. Run `relnav verify` for the error table.
5. Edit into a 60–90 s video following the storyboard: hook, tracking, pins, relative readouts, return-to-pin, proof, architecture flash.
