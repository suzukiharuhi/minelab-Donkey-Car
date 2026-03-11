import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rospy

from .config import LoggingConfig
from .csv_logger import CsvLogger
from .event_logger import EventLogger
from .image_logger import ImageLogger
from .state_transition_logger import StateTransitionLogger

class SessionLogger:
    """
    1走行分のログを管理するメインクラス        

    Usage:
        logger = SessionLogger(config)
        logger.start_session()

        # 各状態から使用
        logger.log_event("state_changed", {"from": "idle", "to": "follow_row"})
        logger.log_image(frame_id, rgb_image, tag="aruco_detect", state="follow_row")
        logger.log_metric(frame_id, {"speed": 0.3, "angle": 0.1}, state="follow_row")

        logger.end_session()
    """

    _instance: Optional["SessionLogger"] = None
    _lock = threading.Lock()

    def __init__(self, config: Optional[LoggingConfig] = None):
        self._config = config or LoggingConfig()
        self._session_dir: Optional[Path] = None
        self._start_time: Optional[datetime] = None
        self._frame_counter: int = 1
        self._frame_lock = threading.Lock()
        self._active = False

        # サブロガー
        self._image_logger: Optional[ImageLogger] = None
        self._csv_logger: Optional[CsvLogger] = None
        self._event_logger: Optional[EventLogger] = None
        self._state_transition_logger: Optional[StateTransitionLogger] = None

    # ------------------------------------------------------------------
    # シングルトンアクセス（ROSノードから簡単に参照できるように）
    # システム内でインスタンスを1つだけに制限する設計パターン
    # ------------------------------------------------------------------

    @classmethod
    # インスタンスを取得するクラスメソッド。初期化されていない場合はエラーを投げる。
    def get_instance(cls) -> "SessionLogger":
        if cls._instance is None:
            raise RuntimeError(
                "SessionLogger is not initialized. Call SessionLogger.initialize() first."
            )
        return cls._instance

    @classmethod
    # インスタンスを初期化するクラスメソッド。すでに初期化されている場合は既存のインスタンスを返す。
    def initialize(cls, config: Optional[LoggingConfig] = None) -> "SessionLogger":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls(config)
            return cls._instance

    # ------------------------------------------------------------------
    # セッション管理
    # ------------------------------------------------------------------

    def start_session(self, session_tag: str = "") -> Path:
        """
        走行セッションを開始する．
        ログディレクトリを作成し，サブロガーを初期化する．
        """
        self._start_time = datetime.now()
        dir_name = self._build_session_dir_name(session_tag)
        self._session_dir = self._config.log_root / dir_name
        self._session_dir.mkdir(parents=True, exist_ok=True)

        # サブディレクトリ
        (self._session_dir / "images").mkdir(exist_ok=True)
        (self._session_dir / "plots").mkdir(exist_ok=True)
        (self._session_dir / "states").mkdir(exist_ok=True)
        (self._session_dir / "csv").mkdir(exist_ok=True)

        # サブロガー初期化
        self._image_logger = ImageLogger(self._session_dir, self._config)
        self._csv_logger = CsvLogger(self._session_dir, self._config)
        self._event_logger = EventLogger(self._session_dir)
        self._state_logger = StateTransitionLogger(self._session_dir)

        self._active = True
        self._frame_counter = 0

        # セッション情報の記録
        self._write_session_info(session_tag)

        rospy.loginfo(f"[SessionLogger] Session started: {self._session_dir}")
        return self._session_dir

    def end_session(self, success: bool = True, note: str = "") -> None:
        """走行セッションを終了する．"""
        if not self._active:
            return

        self._active = False
        end_time = datetime.now()
        duration = (end_time - self._start_time).total_seconds()

        # 終了情報をsession_info.jsonに追記
        info_path = self._session_dir / "session_info.json"
        with open(info_path, "r") as f:
            info = json.load(f)
        info.update({
            "end_time": end_time.isoformat(),
            "duration_sec": round(duration, 2),
            "total_frames": self._frame_counter,
            "success": success,
            "end_note": note,
        })
        with open(info_path, "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)

        # サブロガーを閉じる
        if self._csv_logger:
            self._csv_logger.close()
        if self._state_logger:
            self._state_logger.close()
        rospy.loginfo(
            f"[SessionLogger] Session ended. "
            f"duration={duration:.1f}s frames={self._frame_counter}"
        )

    # ------------------------------------------------------------------
    # フレーム番号管理
    # ------------------------------------------------------------------

    def next_frame(self) -> int:
        """フレーム番号をインクリメントして返す．"""
        with self._frame_lock:
            self._frame_counter += 1
            return self._frame_counter

    @property
    def current_frame(self) -> int:
        return self._frame_counter

    def fmt_frame(self, frame_id: int) -> str:
        """フレーム番号をゼロ埋め文字列に変換する．"""
        return str(frame_id).zfill(self._config.frame_digits)

    # ------------------------------------------------------------------
    # ログAPI（各状態から呼び出す）
    # ------------------------------------------------------------------

    def log_event(
        self,
        event_type: str,
        data: Dict[str, Any],
        frame_id: Optional[int] = None,
        state: str = "",
    ) -> None:
        """
        イベント（状態遷移，エラー，マーカー検出など）を記録する．

        Args:
            event_type: イベントの種類 ("state_changed", "marker_detected", ...)
            data:       イベントに付随するデータ
            frame_id:   フレーム番号（省略時は現在フレーム）
            state:      発生した状態名
        """
        if not self._active:
            return
        fid = frame_id if frame_id is not None else self._frame_counter
        self._event_logger.write(fid, event_type, state, data)

    def log_metric(
        self,
        frame_id: int,
        metrics: Dict[str, float],
        state: str = "",
        tag: str = "metrics",
    ) -> None:
        """
        数値メトリクス（速度，角度，距離など）をCSVに記録する．

        Args:
            frame_id: フレーム番号
            metrics:  記録する数値の辞書
            state:    発生した状態名
            tag:      メトリクスの種類識別子
        """
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(frame_id, tag, state, metrics)
    
    def log_cpu_metrics(self, frame_id: int, temp_c: float) -> None:
        """CPU温度のシステムメトリクスを記録するヘルパー"""
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(
            frame_id=frame_id,
            tag="cpu",
            state="",
            data={"temp_c": temp_c},
        )
    
    def log_battery_metrics(self, frame_id: int, voltage_v: float, percent: float) -> None:
        """バッテリーメトリクスを記録するヘルパー"""
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(
            frame_id=frame_id,
            tag="battery",
            state="",
            data={"voltage_v": voltage_v, "percent": percent},
        )

    def log_csv_row(
        self,
        frame_id: int,
        tag: str,
        data: Dict[str, Any],
        state: str = "",
    ) -> None:
        """
        任意の辞書1行をCSVに記録する汎用API.

        例:
            tag="cpu"        -> csv/metrics_cpu.csv
            tag="battery"    -> csv/metrics_battery.csv
            tag="debug_pose" -> csv/metrics_debug_pose.csv
        """
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(frame_id, tag, state, data)

    def log_image(
        self,
        frame_id: int,
        image,
        tag: str = "frame",
        state: str = "",
        subdir: Optional[str] = None,
    ) -> Optional[Path]:
        """
        画像を保存する．

        Args:
            frame_id: フレーム番号
            image:    numpy配列 (H, W, C) または (H, W)
            tag:      画像の種類識別子 (例: "aruco_detect", "depth_map")
            state:    発生した状態名
            subdir:   images/以下のサブディレクトリ（省略時はstate名）
        Returns:
            保存したファイルパス
        """
        if not self._active or not self._config.save_images:
            return None
        return self._image_logger.save(frame_id, image, tag, state, subdir)

    def log_raw_image(self, frame_id: int, image: Any, state: str = "", tag: str = "raw") -> Optional[Path]:
        """生のRGB画像を images/raw_images/ に保存するヘルパー.

        保存先: self._session_dir/images/raw_images/
        ファイル名: フレーム番号 + tag
        """
        if not self._active or not self._config.save_images:
            return None
        # subdir="raw_images" を明示して、常に images/raw_images/ 配下に保存
        return self._image_logger.save(
            frame_id=frame_id,
            image=image,
            tag=tag,
            state=state,
            subdir="raw_images",
        )

    def log_plot(
        self,
        frame_id: int,
        fig,
        tag: str = "plot",
        state: str = "",
    ) -> Optional[Path]:
        """
        Matplotlibのfigureを保存する．

        Args:
            frame_id: フレーム番号
            fig:      matplotlib.figure.Figure
            tag:      プロットの種類識別子
            state:    発生した状態名
        Returns:
            保存したファイルパス
        """
        if not self._active or not self._config.save_plots:
            return None
        return self._image_logger.save_figure(frame_id, fig, tag, state)
    
    def log_state_transition(
        self,
        frame_id: int,
        from_state: str,
        to_state: str,
        reason: str = "",
    ) -> None:
        """
        状態遷移履歴を state_transitions.csv に記録する.

        Args:
            frame_id:   遷移したフレーム番号
            from_state: 遷移元状態名
            to_state:   遷移先状態名
            reason:     任意のメモ（トリガ要因など）
        """
        if not self._active or self._state_logger is None:
            return
        try:
            import rospy
            t_ros = rospy.Time.now().to_sec()
        except Exception:
            t_ros = 0.0
        self._state_logger.write(
            frame_id=frame_id,
            from_state=from_state,
            to_state=to_state,
            reason=reason,
            t_ros=t_ros,
        )

    def log_json(
        self,
        frame_id: int,
        data: Dict[str, Any],
        tag: str = "data",
        state: str = "",
    ) -> Optional[Path]:
        """
        任意のデータをJSONとして保存する．

        Args:
            frame_id: フレーム番号
            data:     保存するデータ辞書
            tag:      データの種類識別子
            state:    発生した状態名
        Returns:
            保存したファイルパス
        """
        if not self._active:
            return None
        state_dir = self._session_dir / "states" / state if state else self._session_dir / "states"
        state_dir.mkdir(parents=True, exist_ok=True)
        fname = f"{self.fmt_frame(frame_id)}_{tag}.json"
        fpath = state_dir / fname
        with open(fpath, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=str)
        return fpath

    # ------------------------------------------------------------------
    # 内部ヘルパー
    # ------------------------------------------------------------------

    def _build_session_dir_name(self, tag: str) -> str:
        ts = self._start_time.strftime("%Y-%m-%d_%H-%M-%S")
        return f"{ts}_{tag}" if tag else ts

    def _write_session_info(self, tag: str) -> None:
        info = {
            "start_time": self._start_time.isoformat(),
            "session_tag": tag,
            "config": {
                "save_images": self._config.save_images,
                "save_csv": self._config.save_csv,
                "save_plots": self._config.save_plots,
                "verbose": self._config.verbose,
            },
        }
        with open(self._session_dir / "session_info.json", "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)