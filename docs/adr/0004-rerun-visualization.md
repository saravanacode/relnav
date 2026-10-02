# ADR 0004: Rerun for the 3D view

**Status:** accepted (2026-10-02)

**Context.** The demo needs a 3D world with camera, path, map points and pins, plus a
timeline to scrub.

**Decision.** Rerun 0.38. The viewer runs as the normal user; the sudo process connects over
gRPC. Keyframe clouds are logged once in their camera frame; loop closure re-logs only
keyframe transforms.

**Consequences.** Good-looking, scrubbable views with little code. Rerun's API changes
between minor versions; pin the version in `uv.lock`.
