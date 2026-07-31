"""Configuration loading and path helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"configuration root must be a mapping: {config_path}")
    return data


def load_json(path: str | Path) -> dict[str, Any]:
    calibration_path = Path(path).resolve()
    data = json.loads(calibration_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(
            f"calibration root must be an object: {calibration_path}"
        )
    return data
