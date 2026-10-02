# ADR 0005: Infrared projector off by default

**Status:** accepted (2026-10-02)

**Context.** The D435i projects an IR dot pattern. The dots move with the camera, so features
on them look static in the image and corrupt visual odometry.

**Decision.** Emitter off by default (`camera.emitter_enabled: false`).

**Consequences.** Clean tracking features. Depth is weaker on plain surfaces, so pins must
target textured spots. Emitter on/off interleaving is a later option.
