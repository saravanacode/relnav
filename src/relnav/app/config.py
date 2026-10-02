"""Application configuration: YAML validated by pydantic into the components' dataclasses."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict

from relnav.adapters.realsense.config import RealSenseConfig
from relnav.core.pipeline import SlamConfig


class AppConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    camera: RealSenseConfig = RealSenseConfig()
    slam: SlamConfig = SlamConfig()
    rerun_url: str | None = None
    rerun_spawn: bool = False


def load_config(path: Path | None) -> AppConfig:
    if path is None:
        return AppConfig()
    data = yaml.safe_load(path.read_text()) or {}
    return AppConfig.model_validate(data)
