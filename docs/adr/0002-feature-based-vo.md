# ADR 0002: Feature-based visual odometry (ORB + PnP)

**Status:** accepted (2026-10-02)

**Context.** Options were our own ORB + PnP odometry, Open3D's dense RGB-D odometry, or
wrapping ORB-SLAM3 / RTAB-Map (which do not build or run well on macOS).

**Decision.** Own pipeline: grid-spread ORB on the left IR image, 3D points from aligned
depth, PnP + RANSAC against the reference keyframe, keyframes every 15 cm / 15°, proximity
loop closure verified by PnP, Open3D pose-graph optimization.

**Consequences.** Fast on CPU, fully testable, and a stronger demo story. Weak on textureless
walls; mitigated by the feature grid. Measured during planning: ~0.44 cm PnP error per
keyframe step at 848×480 on the synthetic room.
