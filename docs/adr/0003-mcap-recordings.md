# ADR 0003: MCAP for recordings

**Status:** accepted (2026-10-02)

**Context.** The demo uses record-then-replay, and replay must be deterministic (REQ-REP-01).
A 2-minute walk is ~4 GB raw.

**Decision.** MCAP with zstd chunks and a small versioned encoding (`relnav.v1`): raw uint8
IR, uint16 depth units, JSON intrinsics and commands. Commands are written before their
frame. Replay streams the file, so a recording cut off mid-write replays up to the cut.

**Consequences.** Standard robotics container that ROS 2 tooling understands. Not yet
viewable in Foxglove, because the encoding is custom; switching to Foxglove schemas is a
later option.
