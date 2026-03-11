import csv
from datetime import datetime
from pathlib import Path
from typing import Optional


class StateTransitionLogger:
    """状態遷移履歴を state_transitions.csv に記録するロガー."""

    def __init__(self, session_dir: Path):
        self._path = session_dir / "state_transitions.csv"
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "t_wall",       # 壁時計 (ISO8601)
                "t_ros",        # ROS 時刻 [sec]
                "frame_id",     # 遷移が起きたフレーム番号
                "from_state",   # 遷移元状態
                "to_state",     # 遷移先状態
                "reason",       # 任意の理由/メモ
            ],
        )
        self._writer.writeheader()

    def write(
        self,
        frame_id: int,
        from_state: str,
        to_state: str,
        reason: str = "",
        t_ros: Optional[float] = None,
    ) -> None:
        now = datetime.now().isoformat()
        self._writer.writerow(
            {
                "t_wall": now,
                "t_ros": float(t_ros) if t_ros is not None else 0.0,
                "frame_id": int(frame_id),
                "from_state": str(from_state),
                "to_state": str(to_state),
                "reason": str(reason),
            }
        )
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()