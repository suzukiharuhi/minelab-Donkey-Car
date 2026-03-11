"""Metric logger (ROS-free).

Lightweight wrapper that appends key-value metrics rows to a CSV file.
Intended as a standalone component when ``SessionLogger`` is too heavy.
"""
from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class MetricLogger:
    """Append ``{ts, **metrics}`` rows to a CSV file.

    Args:
        path:    Full path to the output ``.csv`` file.
        buffer:  Number of rows to buffer before flushing (default 1 = immediate).
    """

    def __init__(self, path: str, buffer: int = 1) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._buffer = buffer
        self._rows: List[Dict[str, Any]] = []
        self._writer: Optional[csv.DictWriter] = None
        self._fh = open(self._path, "w", newline="", encoding="utf-8")  # noqa: WPS515

    def log(self, metrics: Dict[str, Any]) -> None:
        row = {"ts": time.time(), **metrics}
        self._rows.append(row)
        if len(self._rows) >= self._buffer:
            self._flush()

    def _flush(self) -> None:
        if not self._rows:
            return
        if self._writer is None:
            self._writer = csv.DictWriter(
                self._fh, fieldnames=list(self._rows[0].keys())
            )
            self._writer.writeheader()
        for row in self._rows:
            try:
                self._writer.writerow(row)
            except ValueError:
                pass
        self._fh.flush()
        self._rows.clear()

    def close(self) -> None:
        self._flush()
        self._fh.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
