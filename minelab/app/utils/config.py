"""YAML configuration loader (ROS-free).

Provides a simple helper to load YAML files into plain Python dicts.
No ROS parameter server is involved.
"""
from __future__ import annotations

import logging
import os
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False
    logger.warning("PyYAML not installed; config loading will not work.")


def load_yaml(path: str) -> Dict[str, Any]:
    """Load a YAML file and return its contents as a dict.

    Args:
        path:  Absolute or relative path to the ``.yaml`` file.

    Returns:
        Parsed YAML contents, or an empty dict on error.
    """
    if not _YAML_AVAILABLE:
        logger.error("PyYAML is required but not installed.")
        return {}

    if not os.path.isfile(path):
        logger.error("Config file not found: %s", path)
        return {}

    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        logger.exception("Failed to load config from %s: %s", path, exc)
        return {}


def load_configs(*paths: str) -> Dict[str, Any]:
    """Load and merge multiple YAML files.  Later files take precedence.

    Args:
        *paths: One or more paths to ``.yaml`` config files.

    Returns:
        Merged dictionary.
    """
    merged: Dict[str, Any] = {}
    for p in paths:
        merged.update(load_yaml(p))
    return merged


class ConfigNamespace:
    """Thin wrapper around a config dict that supports attribute access.

    Example::

        cfg = ConfigNamespace(load_yaml("pilot.yaml"))
        hz = cfg.get("loop_hz", 50)
    """

    def __init__(self, data: Dict[str, Any]) -> None:
        self._data = data

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self._data[key]

    def __contains__(self, key: str) -> bool:
        return key in self._data
