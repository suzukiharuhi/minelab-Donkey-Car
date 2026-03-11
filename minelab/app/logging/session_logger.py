"""Session logger (ROS-free).

Simple disk logger that writes JSON-Lines records and optional CSV metrics.
Uses the standard ``logging`` module; no ROS dependency.
"""
from __future__ import annotations

import csv
import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class SessionLogger:
    """Write structured records to a per-session directory.

    Args:
        log_root:   Root directory where session folders are created.
        session_tag: Human-readable label for the session (e.g. ``"run_01"``).
        save_csv:    Whether to write a ``metrics.csv`` file.
        verbose:     Log each written record to stderr.
    """

    def __init__(
        self,
        log_root: str = "/tmp/minelab_logs",
        session_tag: str = "session",
        save_csv: bool = True,
        verbose: bool = False,
    ) -> None:
        self._log_root = Path(log_root)
        self._tag = session_tag
        self._save_csv = save_csv
        self._verbose = verbose

        ts = time.strftime("%Y%m%d_%H%M%S")
        self._session_dir = self._log_root / f"{ts}_{session_tag}"
        self._session_dir.mkdir(parents=True, exist_ok=True)

        self._jsonl_path = self._session_dir / "events.jsonl"
        self._csv_path = self._session_dir / "metrics.csv"

        self._lock = threading.Lock()
        self._csv_writer: Optional[csv.DictWriter] = None
        self._csv_file = None

        if self._save_csv:
            self._csv_file = open(self._csv_path, "w", newline="", encoding="utf-8")  # noqa: WPS515

        logger.info("[SessionLogger] Session directory: %s", self._session_dir)

    # ------------------------------------------------------------------

    def log_event(self, event_type: str, data: Dict[str, Any], state: str = "") -> None:
        """Append a JSON record to ``events.jsonl``."""
        record = {
            "ts": time.time(),
            "state": state,
            "event": event_type,
            **data,
        }
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            with open(self._jsonl_path, "a", encoding="utf-8") as fh:
                fh.write(line + "\n")
        if self._verbose:
            logger.debug("[SessionLogger] event: %s", line)

    def log_metric(
        self,
        metrics: Dict[str, Any],
        state: str = "",
        tag: str = "metrics",
    ) -> None:
        """Append a metrics row to ``metrics.csv``."""
        if not self._save_csv:
            return
        row = {"ts": time.time(), "state": state, "tag": tag, **metrics}
        with self._lock:
            if self._csv_writer is None:
                self._csv_writer = csv.DictWriter(
                    self._csv_file, fieldnames=list(row.keys())
                )
                self._csv_writer.writeheader()
            try:
                self._csv_writer.writerow(row)
            except ValueError:
                pass  # new columns added after header was written
            self._csv_file.flush()

    def log_state_transition(self, from_state: str, to_state: str) -> None:
        self.log_event(
            "state_transition",
            {"from": from_state, "to": to_state},
        )

    def close(self) -> None:
        """Flush and close all open file handles."""
        with self._lock:
            if self._csv_file is not None:
                self._csv_file.close()
                self._csv_file = None
        logger.info("[SessionLogger] Session closed: %s", self._session_dir)

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
