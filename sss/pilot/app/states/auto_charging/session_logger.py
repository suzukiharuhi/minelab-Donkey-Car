import os
import csv
import datetime
from typing import Optional, Dict, Any


class AutoChargeSessionLogger:
    """ログ記録ユーティリティ
    セッションごとにドッキング・充電の試行回数、成功・失敗を記録し、CSVに保存する。
    """

    def __init__(self, log_dir: str = "/home/pi/ugv_ws/logging/charge") -> None:
        self.log_dir = os.path.expanduser(log_dir)
        self._current: Optional[Dict[str, Any]] = None
        os.makedirs(self.log_dir, exist_ok=True)

    def start_session(self) -> None:
        ts = datetime.datetime.now().isoformat()
        self._current = {
            "session_start": ts,
            "docking_attempts": 0,
            "docking_retries": 0,
            "docking_success": False,
            "charging_attempts": 0,
            "charging_retries": 0,
            "charging_success": False,
        }

    def _ensure_session(self) -> bool:
        if self._current is None:
            # Avoid raising; simply ignore if session not started.
            return False
        return True

    def record_docking_attempt(self) -> None:
        if not self._ensure_session():
            return
        self._current["docking_attempts"] += 1

    def record_docking_failure(self) -> None:
        if not self._ensure_session():
            return
        self._current["docking_retries"] += 1

    def record_docking_success(self) -> None:
        if not self._ensure_session():
            return
        self._current["docking_success"] = True

    def record_charging_attempt(self) -> None:
        if not self._ensure_session():
            return
        self._current["charging_attempts"] += 1

    def record_charging_failure(self) -> None:
        if not self._ensure_session():
            return
        self._current["charging_retries"] += 1

    def record_charging_success(self) -> None:
        if not self._ensure_session():
            return
        self._current["charging_success"] = True

    def finalize_session(self) -> Optional[str]:
        if not self._ensure_session():
            return None
        row = self._current.copy()
        # Derive attempts if missing (ensure >= retries)
        if row["docking_attempts"] < row["docking_retries"]:
            row["docking_attempts"] = row["docking_retries"]
        if row["charging_attempts"] < row["charging_retries"]:
            row["charging_attempts"] = row["charging_retries"]
        # Guarantee attempts reflect retries + success attempt
        if row["docking_success"]:
            row["docking_attempts"] = max(row["docking_attempts"], row["docking_retries"] + 1)
        if row["charging_success"]:
            row["charging_attempts"] = max(row["charging_attempts"], row["charging_retries"] + 1)

        csv_path = os.path.join(self.log_dir, "auto_charge_sessions.csv")
        write_header = not os.path.exists(csv_path)
        header = [
            "session_start",
            "docking_attempts",
            "docking_retries",
            "docking_success",
            "charging_attempts",
            "charging_retries",
            "charging_success",
        ]
        try:
            with open(csv_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=header)
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
        except Exception as exc:
            print(f"[AUTO_CHG][LOGGER] Failed to write session log: {exc}")
            return None
        self._current = None
        return csv_path


_logger = AutoChargeSessionLogger()


def get_logger() -> AutoChargeSessionLogger:
    return _logger
