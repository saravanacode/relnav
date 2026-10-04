# relnav status

Last updated: 2026-10-04.

## Where things stand

- Branch `feat/relnav-v0.1` (29 commits ahead of `main`; `main` holds only the spec and
  plan). Not merged, not tagged, not pushed anywhere.
- All 20 plan tasks are built test-first. `uv run pytest -m "not hardware"` → 132 passed;
  ruff, pyright (0 errors) and import-linter are clean. A fresh clone installs and runs.
- A fresh-context reviewer checked the whole branch; 1 Critical and 5 Important findings
  were fixed (each with a test that failed first), plus 2 Minor ones re-graded to
  Important. The plan workspace ledger with every ruling is in
  `.superpowers/sdd/2026-10-02-relnav-implementation/progress.md` (git-ignored).
- The synthetic demo works: `recordings/synthetic.mcap` replays through the full pipeline
  into the HUD and Rerun (300 frames, 3 pins, loop closed, 32 keyframes, 44 fps).
- After the review, the synthetic replay exposed a keyframe bug (a keyframe and a loop
  closure on every frame, 23.6 fps). Fixed in `844558a` by measuring overlap against the
  first-track inliers; spec §8.3 amended.

## Requirements

| ID | Status |
|---|---|
| REQ-ACC-03 trajectory RMSE < 2 cm (synthetic) | Passing |
| REQ-REL-01 timeouts / failed starts never crash | Passing |
| REQ-REL-02 LOST then recovers | Passing |
| REQ-REP-01 identical replay | Passing |
| REQ-ARCH-01 hardware-free core | Passing |
| REQ-PERF-01 ≥ 25 fps | 44 fps on synthetic replay; real recording not yet measured |
| REQ-ACC-01 pin-to-pin < 3 cm (real room) | Waiting for the field test |
| REQ-ACC-02 pin consistent after a 10 m loop | Waiting for the field test |

## Blocker: the camera's USB link

The camera keeps connecting at USB 2 (480 Mb/s) because of the cable/plug. The user does
not have a USB 3 cable yet. Buy: USB-C to USB-C, USB 3.1/3.2 (5 Gb/s or more) or
Thunderbolt 3/4, 1 m or shorter. (The white MacBook charging cable is USB 2 only.)

Evidence so far:
- Two live runs on 2026-10-02 at USB 2 with the default 848×480 @ 30 config:
  `results/live-20261002-163257` (27 frames) and `results/live-20261002-165446`
  (192 frames). Both ended with
  `source error: camera stopped delivering frames: wait_for_frames cannot be called before start()`,
  i.e. the camera stopped mid-run. The second run predates the keyframe fix (80 keyframes,
  67 loop closures in 192 frames).
- `config/usb2.yaml` (640×480 @ 15 fps) is valid (the camera lists the mode and
  `can_resolve` is True on USB 2), but starting the stream failed 4/4 times with
  `failed to set power state`. Likely cause: USB 2 power budget (2.48 W). Not yet
  retried after a fresh replug.

## Next steps

1. Get the USB 3 cable; confirm `Link Speed: 5 Gb/s`.
2. Hardware smoke test:
   `sudo env RELNAV_HARDWARE=1 PYTHONDONTWRITEBYTECODE=1 "$(command -v uv)" run --no-sync pytest -m hardware -q`
3. Live run: `uv run relnav viewer`, then `scripts/relnav-sudo live --name first-live`.
   Check the HUD opens under sudo, tracking reaches OK, pins appear, and
   `results/first-live/` is owned by the user.
4. Field test per `docs/field-test.md` (record, replay, `verify`, fps check, pin
   consistency after the loop).
5. Tag `v0.1.0`, merge `feat/relnav-v0.1` into `main`, push to GitHub.
6. Record the LinkedIn video (storyboard in spec §16): screen-record HUD + Rerun during
   `replay --realtime`, film the tape-measure check, end with the architecture diagram.

## Known minor issues (deferred from the review)

- If the capture thread dies outside `wait_for_frames`, the run ends as "no frames for 5
  attempts" without the real cause.
- A recorded pin pixel outside the image reads the wrong depth patch (only reachable by
  hand-editing a recording).
- Truncated recordings and record-mode dropped frames are not written to `summary.json`.
- The HUD shows pin azimuth only, not elevation.
- Malformed verify YAML / `pins.json` / `--config` files give tracebacks.
- If the HUD or Rerun fails to start, the camera and recorder are not closed.
- Rerun cloud colours use the pre-loop-closure pose; the full path is re-logged each frame.
- `start_with_retry` also retries non-transient errors such as "Failed to resolve the
  request" (found while testing USB 2).

## Related learning scripts

`~/self/slam` holds the earlier learning projects (camera check, live depth, distance
meter, depth filters, point clouds, box measurer). Its `vendor/pyrealsense2` is where the
bindings were first built.
