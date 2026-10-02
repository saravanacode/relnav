"""relnav.v1 message encoding for MCAP: small binary headers plus raw pixels, JSON metadata."""

from __future__ import annotations

import dataclasses
import json
import struct

import numpy as np
from numpy.typing import DTypeLike, NDArray

from relnav.domain.commands import Command, DropPin
from relnav.domain.types import Intrinsics

ENCODING = "relnav.v1"
TOPIC_INFO = "/camera/info"
TOPIC_IR = "/camera/ir"
TOPIC_DEPTH = "/camera/depth"
TOPIC_COMMANDS = "/commands"
SCHEMA_DOC = (
    b"relnav.v1. /camera/info: JSON Intrinsics. /camera/ir, /camera/depth: little-endian "
    b"header (int64 frame index, uint32 width, uint32 height) then raw uint8 IR or uint16 "
    b"depth units (meters = units * depth_scale). /commands: JSON command."
)
_HEADER = struct.Struct("<qII")


def encode_info(intrinsics: Intrinsics) -> bytes:
    return json.dumps(dataclasses.asdict(intrinsics)).encode()


def decode_info(data: bytes) -> Intrinsics:
    return Intrinsics(**json.loads(data))


def encode_image(index: int, image: NDArray) -> bytes:
    height, width = image.shape
    return _HEADER.pack(index, width, height) + np.ascontiguousarray(image).tobytes()


def decode_image(data: bytes, dtype: DTypeLike) -> tuple[int, NDArray]:
    index, width, height = _HEADER.unpack_from(data)
    image = np.frombuffer(data, dtype=dtype, offset=_HEADER.size).reshape(height, width)
    return index, image.copy()


def depth_to_units(depth_m: NDArray[np.float32], depth_scale: float) -> NDArray[np.uint16]:
    return np.clip(np.rint(depth_m / depth_scale), 0, 65535).astype(np.uint16)


def units_to_depth(units: NDArray[np.uint16], depth_scale: float) -> NDArray[np.float32]:
    return units.astype(np.float32) * np.float32(depth_scale)


def encode_command(command: DropPin) -> bytes:
    data = {"type": "drop_pin", "label": command.label, "u": command.u, "v": command.v}
    return json.dumps(data).encode()


def decode_command(data: bytes) -> Command:
    payload = json.loads(data)
    if payload.get("type") != "drop_pin":
        raise ValueError(f"unknown command {payload!r}")
    return DropPin(label=payload.get("label"), u=payload.get("u"), v=payload.get("v"))
