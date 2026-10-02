# Field test: real room, real camera

Run this once the code is complete, to verify REQ-ACC-01, REQ-ACC-02 and REQ-PERF-01 and to
capture the demo footage.

## 0. Prepare

1. Plug the D435i straight into a Mac USB-C port, quickly and firmly. Check the link:
   `system_profiler SPUSBHostDataType | grep -A10 RealSense | grep "Link Speed"` → `5 Gb/s`.
2. Pick three pin targets 1–3 m apart with visible texture or edges (door frame, chair
   back, laptop edge). Plain painted surfaces give no depth with the emitter off.
3. Tape-measure the target-to-target distances and write `recordings/truth.yaml`:

   ```yaml
   pairs:
     - {a: DOOR, b: CHAIR, distance_m: 2.43}
     - {a: DOOR, b: LAPTOP, distance_m: 3.10}
     - {a: CHAIR, b: LAPTOP, distance_m: 1.75}
   ```

## 1. Record a walk

```bash
scripts/relnav-sudo record recordings/home.mcap
```

Hold the camera at chest height. Aim the crosshair at the DOOR target and press `p`, then do
the same for CHAIR and LAPTOP (labels follow `config/default.yaml`). Walk a slow loop of about
10 m around the room and come back to look at the DOOR again. Press `q` to stop.

## 2. Watch it replay

In one terminal (normal user): `uv run relnav viewer`
In another: `uv run relnav replay recordings/home.mcap --realtime --name home`

## 3. Check accuracy (REQ-ACC-01)

```bash
uv run relnav verify results/home recordings/truth.yaml
```

Every row must say PASS (error < 3 cm).

## 4. Check speed (REQ-PERF-01)

```bash
uv run relnav replay recordings/home.mcap --no-hud --no-rerun --name speed
uv run python -c "import json; print(json.load(open('results/speed/timing.json'))['fps'])"
```

The printed fps must be ≥ 25.

## 5. Check pin consistency after the loop (REQ-ACC-02)

In the Rerun viewer, scrub the timeline to the moment you look at the DOOR again at the end
of the loop. The DOOR pin must sit on the door: the HUD's DOOR distance should match a tape
measurement from the camera position to within 5 cm.

## 6. Capture the demo

Replay with `--realtime`, place the HUD and Rerun windows side by side, and record the screen
with Cmd+Shift+5. Film the tape-measure check with a phone. Edit together following the
storyboard in the spec, Section 16.
