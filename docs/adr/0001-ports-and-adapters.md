# ADR 0001: Ports and adapters

**Status:** accepted (2026-10-02)

**Context.** relnav must be reusable across projects and sensors, testable without hardware,
and portable later to ROS 2 or embedded boards.

**Decision.** Five layers: domain → core → ports → adapters → app. The core sees the world
only through Protocols (FrameSource, StateSink, CommandSource, Recorder). import-linter
enforces the direction in CI (REQ-ARCH-01).

**Consequences.** A new camera, IMU or ROS 2 bridge is one adapter. Core tests need no
camera. Small cost: a few protocol files, and commands live in `domain` because the core
consumes them.
