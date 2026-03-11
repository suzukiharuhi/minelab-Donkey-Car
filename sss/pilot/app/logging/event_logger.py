import csv
import json
import threading
from pathlib import Path
from typing import Any, Dict


class EventLogger:
    """
    状態遷移・離散イベントをCSVに記録するロガー.

    events.csv の列:
        frame_id, timestamp_ros, state, event_type, data_json
    """

    _FIELDNAMES = ["frame_id", "timestamp_ros", "state", "event_type", "data_json"]

    def __init__(self, session_dir: Path):
        self._fpath = session_dir / "events.csv"
        self._lock = threading.Lock()
        self._file = open(self._fpath, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self._FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def write(
        self,
        frame_id: int,
        event_type: str,
        state: str,
        data: Dict[str, Any],
    ) -> None:
        import rospy

        row = {
            "frame_id": frame_id,
            "timestamp_ros": rospy.get_time(),
            "state": state,
            "event_type": event_type,
            "data_json": json.dumps(data, ensure_ascii=False, default=str),
        }
        with self._lock:
            self._writer.writerow(row)
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            if self._file:
                self._file.close()