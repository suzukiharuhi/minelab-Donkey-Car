"""Step and state CSV logging utilities for pilot.

- 共通ステップログ: /logging/common/
- 状態別ログ:       /logging/states/<STATE_NAME>/

ログルートは環境変数 UGV_LOG_DIR で上書き可能。
"""
from __future__ import annotations

import os
import csv
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, List, Dict, Optional

from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand
from pilot.app.utils.data_models import IMUData, CameraData, ArUcoMarker


_LOG_ROOT_ENV = "UGV_LOG_DIR"
_DEFAULT_LOG_ROOT = "/home/pi/ugv_ws/logging"


def _get_log_root() -> Path:
    # base = os.environ.get(_DEFAULT_LOG_ROOT)
    return Path(_DEFAULT_LOG_ROOT)


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


@dataclass
class _CsvLogger:
    path: Path
    header: List[str]

    def __post_init__(self) -> None:
        _ensure_dir(self.path.parent)
        self._fp = self.path.open("a", newline="")
        self._writer = csv.writer(self._fp)
        # 新規ファイルならヘッダを書き込む
        if self._fp.tell() == 0:
            self._writer.writerow(self.header)
            self._fp.flush()

    def log_row(self, row: Iterable[Any]) -> None:
        self._writer.writerow(list(row))
        self._fp.flush()


_common_logger: Optional[_CsvLogger] = None
_state_loggers: Dict[str, _CsvLogger] = {}


def _new_timestamp_str() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _get_common_logger() -> _CsvLogger:
    global _common_logger
    if _common_logger is None:
        root = _get_log_root() / "common"
        _ensure_dir(root)
        ts = _new_timestamp_str()
        path = root / f"steps_{ts}.csv"
        header = [
            "timestamp",
            "state",
            "next_state",
            # "has_imu",
            # "imu_yaw_deg",
            # "has_camera",
            # "num_markers",
            "command_type",
            "speed_level",
        ]
        _common_logger = _CsvLogger(path=path, header=header)
    return _common_logger


def get_state_logger(state_name: str, header: Iterable[str]) -> _CsvLogger:
    """状態別 CSV ロガーを取得（なければ作成）。"""
    key = state_name
    logger = _state_loggers.get(key)
    if logger is None:
        root = _get_log_root() / "states" / state_name
        _ensure_dir(root)
        ts = _new_timestamp_str()
        path = root / f"{state_name}_{ts}.csv"
        logger = _CsvLogger(path=path, header=list(header))
        _state_loggers[key] = logger
    return logger


def log_common_step(
    state: StateType,
    command: Optional[ControlCommand],
    # imu_data: Optional[IMUData],
    # camera_data: Optional[CameraData],
    # markers: Optional[List[ArUcoMarker]],
    next_state: Optional[StateType] = None,
) -> None:
    """1ステップごとの共通ログを CSV に書き出す。"""
    logger = _get_common_logger()

    # # 時刻は IMU→カメラ→現在時刻の優先順で使用
    # if imu_data is not None and getattr(imu_data, "timestamp", None) is not None:
    #     ts = float(imu_data.timestamp)
    # elif camera_data is not None and getattr(camera_data, "timestamp", None) is not None:
    #     ts = float(camera_data.timestamp)
    # else:
    #     ts = time.time()
    ts = time.time()

    # has_imu = int(imu_data is not None)
    # imu_yaw = ""
    # if imu_data is not None and getattr(imu_data, "euler_angles", None) is not None:
    #     try:
    #         imu_yaw = float(imu_data.euler_angles[2])
    #     except Exception:
    #         imu_yaw = ""

    # has_camera = int(camera_data is not None)
    # num_markers = len(markers) if markers is not None else 0

    cmd_type = ""
    speed_level = ""
    if command is not None:
        cmd_type = getattr(getattr(command, "command", None), "name", "")
        spd = getattr(command, "speed", None)
        speed_level = getattr(spd, "name", "") if spd is not None else ""

    row = [
        f"{ts:.6f}",
        getattr(state, "name", str(state)),
        getattr(next_state, "name", "") if next_state is not None else "",
        # has_imu,
        # imu_yaw,
        # has_camera,
        # num_markers,
        cmd_type,
        speed_level,
    ]
    try:
        logger.log_row(row)
    except Exception:
        # ログ失敗時も制御自体は継続させたいので握りつぶす
        pass
