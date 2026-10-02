from relnav.core.health import HealthConfig, HealthMonitor
from relnav.core.tracker import TrackResult
from relnav.domain.se3 import SE3
from relnav.domain.types import TrackingState

GOOD = TrackResult(True, SE3.identity(), inliers=200, rmse_px=0.8)
FEW = TrackResult(True, SE3.identity(), inliers=40, rmse_px=0.8)
BLURRY = TrackResult(True, SE3.identity(), inliers=200, rmse_px=2.5)
FAIL = TrackResult.failed()


def started() -> HealthMonitor:
    monitor = HealthMonitor(HealthConfig())
    monitor.initialized()
    return monitor


def test_starts_initializing_then_ok():
    monitor = HealthMonitor(HealthConfig())
    assert monitor.state is TrackingState.INITIALIZING
    assert monitor.initialized().state is TrackingState.OK


def test_quality_sets_ok_or_degraded():
    monitor = started()
    assert monitor.update(GOOD).state is TrackingState.OK
    assert monitor.update(FEW).state is TrackingState.DEGRADED
    assert monitor.update(BLURRY).state is TrackingState.DEGRADED
    status = monitor.update(GOOD)
    assert status.state is TrackingState.OK and status.inliers == 200


def test_lost_after_three_consecutive_failures():
    monitor = started()
    assert monitor.update(FAIL).state is TrackingState.DEGRADED
    assert monitor.update(FAIL).state is TrackingState.DEGRADED
    assert monitor.update(FAIL).state is TrackingState.LOST


def test_a_success_resets_the_failure_count():
    monitor = started()
    monitor.update(FAIL)
    monitor.update(FAIL)
    monitor.update(GOOD)
    assert monitor.update(FAIL).state is TrackingState.DEGRADED


def test_relocalized_leaves_lost():
    monitor = started()
    for _ in range(3):
        monitor.update(FAIL)
    assert monitor.still_lost().state is TrackingState.LOST
    status = monitor.relocalized(GOOD)
    assert status.state is TrackingState.OK and status.message == "relocalized"
