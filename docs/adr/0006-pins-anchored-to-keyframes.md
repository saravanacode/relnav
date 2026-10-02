# ADR 0006: Pins anchored to keyframes

**Status:** accepted (2026-10-02)

**Context.** Loop closure corrects keyframe poses. A pin stored in world coordinates would
stay where the drifted map put it.

**Decision.** Store each pin in its reference keyframe's frame; its world position is
`T_world_kf · p_kf`, computed on demand.

**Consequences.** Pins snap back onto their objects after loop closure, which is the
"walk back and the pin is still there" shot in the demo.
