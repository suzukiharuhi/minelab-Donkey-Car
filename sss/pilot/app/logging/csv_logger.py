import csv
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from .config import LoggingConfig


class CsvLogger:
    """
    数値メトリクスをCSVファイルに逐次書き出すロガー.

    タグ別に個別のCSVファイルを作成する.
    例:
        metrics_follow_row.csv
        metrics_auto_charging.csv
    """

    def __init__(self, session_dir: Path, config: LoggingConfig):
        self._session_dir = session_dir / "csv"
        self._config = config
        self._writers: Dict[str, _CsvWriter] = {}
        self._lock = threading.Lock()

    def write(
        self,
        frame_id: int,  # フレーム番号
        tag: str,       # メトリクスタグ（ファイル名として使用）
        state: str,     # 状態名（タグに含めるか、別列にするかは設計次第）
        data: Dict[str, Any],
    ) -> None:
        """
        1行分のメトリクスを書き出す.

        自動的に frame_id, timestamp_ros, state 列が先頭に追加される.
        """
        import rospy

        key = f"{state}_{tag}" if state else tag
        row = {
            "frame_id": frame_id,
            "timestamp_ros": rospy.get_time(),
            "state": state,
            **data,
        }

        with self._lock:
            if key not in self._writers:
                fname = f"metrics_{key}.csv"
                fpath = self._session_dir / fname
                self._writers[key] = _CsvWriter(fpath, self._config.csv_flush_interval)
            self._writers[key].write(row)

    def close(self) -> None:
        with self._lock:
            for writer in self._writers.values():
                writer.close()
            self._writers.clear()


class _CsvWriter:
    """1ファイル担当の内部ライタークラス."""

    def __init__(self, fpath: Path, flush_interval: int):
        self._fpath = fpath
        self._flush_interval = flush_interval
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._fieldnames = None
        self._row_count = 0

    def write(self, row: Dict[str, Any]) -> None:
        # フィールド定義は最初の行で確定する
        if self._writer is None:
            self._fieldnames = list(row.keys())
            self._file = open(self._fpath, "w", newline="")
            self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames)
            self._writer.writeheader()
        else:
            # 新しいフィールドが追加された場合は無視（安全側）
            row = {k: row.get(k, "") for k in self._fieldnames}

        self._writer.writerow(row)
        self._row_count += 1

        if self._row_count % self._flush_interval == 0:
            self._file.flush()

    def close(self) -> None:
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None