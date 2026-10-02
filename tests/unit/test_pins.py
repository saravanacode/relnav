import numpy as np
import pytest

from relnav.core.features import Features
from relnav.core.keyframes import KeyframeStore
from relnav.core.pins import PinConfig, PinRegistry
from relnav.domain.se3 import SE3
from relnav.domain.types import Frame, Intrinsics

INTR = Intrinsics(width=424, height=240, fx=210.0, fy=210.0, cx=212.0, cy=120.0)


def wall_frame(depth: float = 2.0) -> Frame:
    return Frame(0, 0, np.zeros((240, 424), np.uint8), np.full((240, 424), depth, np.float32), INTR)


def store_with_origin() -> KeyframeStore:
    store = KeyframeStore()
    empty = Features(np.zeros((0, 2), np.float32), np.zeros((0, 32), np.uint8), np.zeros((0, 3)))
    store.add(0, SE3.identity(), empty, np.zeros((0, 3), np.float32))
    return store


def test_pin_at_center_lands_on_the_wall():
    store = store_with_origin()
    registry = PinRegistry(PinConfig(labels=("DOOR",)))
    result = registry.drop(wall_frame(), 212, 120, SE3.identity(), store[0])
    assert result.pin is not None and result.pin.label == "DOOR"
    assert np.allclose(registry.world_position(result.pin, store), [0.0, 0.0, 2.0])


def test_pin_from_a_moved_camera_is_in_world_coordinates():
    store = store_with_origin()
    registry = PinRegistry(PinConfig())
    camera = SE3.from_rt(np.eye(3), [0.5, 0.0, 0.0])
    pin = registry.drop(wall_frame(), 212, 120, camera, store[0]).pin
    assert pin is not None
    assert np.allclose(pin.point_in_keyframe, [0.5, 0.0, 2.0])
    assert np.allclose(registry.world_position(pin, store), [0.5, 0.0, 2.0])


def test_pin_follows_its_keyframe_when_the_map_is_corrected():
    store = store_with_origin()
    registry = PinRegistry(PinConfig())
    pin = registry.drop(wall_frame(), 212, 120, SE3.identity(), store[0]).pin
    assert pin is not None
    store[0].T_world_cam = SE3.from_rt(np.eye(3), [0.0, 0.0, 0.1])  # loop closure moved it
    assert np.allclose(registry.world_position(pin, store), [0.0, 0.0, 2.1])


def test_pin_without_depth_is_rejected():
    store = store_with_origin()
    registry = PinRegistry(PinConfig())
    result = registry.drop(wall_frame(depth=0.0), 212, 120, SE3.identity(), store[0])
    assert result.pin is None and result.reason == "no depth at crosshair"
    assert registry.pins == ()


def test_labels_in_order_then_numbered():
    store = store_with_origin()
    registry = PinRegistry(PinConfig(labels=("DOOR", "CHAIR")))
    labels = []
    for label in (None, None, None, "LAPTOP"):
        pin = registry.drop(wall_frame(), 212, 120, SE3.identity(), store[0], label).pin
        assert pin is not None
        labels.append(pin.label)
    assert labels == ["DOOR", "CHAIR", "P3", "LAPTOP"]


def test_readouts_give_distance_and_bearing():
    store = store_with_origin()
    registry = PinRegistry(PinConfig(labels=("A", "B")))
    registry.drop(wall_frame(), 212, 120, SE3.identity(), store[0])  # (0, 0, 2)
    registry.drop(wall_frame(), 212 + 210, 120, SE3.identity(), store[0])  # (2, 0, 2)
    a, b = registry.readouts(SE3.identity(), store)
    assert a.label == "A" and a.distance_m == pytest.approx(2.0) and a.azimuth_deg == 0.0
    assert b.azimuth_deg == pytest.approx(45.0)
    (pair,) = registry.pairs(store)
    assert (pair.label_a, pair.label_b) == ("A", "B")
    assert pair.distance_m == pytest.approx(2.0)
