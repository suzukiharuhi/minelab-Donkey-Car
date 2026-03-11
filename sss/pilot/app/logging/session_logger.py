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
        self._frame_counter: int = 0
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
        # image_logger は stop() でキューを drain してから終了する
        if self._image_logger:
            self._image_logger.close()
        if self._csv_logger:
            self._csv_logger.close()
        if self._state_logger:
            self._state_logger.close()
        if self._event_logger:
            self._event_logger.close()
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

    def log_event(self, event_type: str, data: Dict[str, Any], state: str = "") -> None:
        """ イベントログ（状態遷移，エラー，マーカー検出など）を記録する．
        Args:
            event_type: イベントの種類識別子（例: "state_changed", "aruco_detected", "error"）
            data:       イベントに関連する任意のデータ辞書
            state:      発生した状態名
        Usage:
            例1) logger.log_event(event_type="state_changed", data={"from": "idle", "to": "follow_row"}, state="follow_row")
            例2) logger.log_event(event_type="aruco_detected", data={"id": 23, "distance": 1.5}, state="follow_row")
            例3) logger.log_event(event_type="error", data={"message": "Camera failure"}, state="follow_row")
        """
        if not self._active:
            return
        frame_id = self._frame_counter
        self._event_logger.write(frame_id, event_type, state, data)

    def log_metric(self, metrics: Dict[str, float], state: str="", tag: str="") -> None:
        """数値メトリクスをCSVに記録する．
        Args:
            state:      メトリクスが関連する状態名
            tag:        メトリクスタグ（例: "follow_row", "auto_charging"）
            metrics:    メトリクスの辞書（例: {"speed": 0.3, "angle": 0.1}）
        Usage:
            例1) logger.log_metric(state="follow_row", tag="follow_row", metrics={"speed": 0.3, "angle": 0.1})
            例2) logger.log_metric(state="auto_charging", tag="auto_charging", metrics={"distance_to_dock": 0.5})
        """
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(
            frame_id=self._frame_counter, 
            tag=tag, 
            state=state, 
            data=metrics
        )
    
    def log_cpu_metrics(self, temp_c: float) -> None:
        """CPU温度のシステムメトリクスを記録するヘルパー
        Args:
            temp_c:     CPU温度
        Usage:
            logger.log_cpu_metrics(temp_c)
        """
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(
            frame_id=self._frame_counter,
            tag="cpu",
            state="",
            data={"temp_c": temp_c},
        )
    
    def log_battery_metrics(self, voltage_v: float, percent: float) -> None:
        """バッテリーメトリクスを記録するヘルパー
        Args:            
            voltage_v:   バッテリー電圧
            percent:     バッテリー残量（0-100%）
        Usage:
            logger.log_battery_metrics(voltage_v, percent)
        """
        if not self._active or not self._config.save_csv:
            return
        self._csv_logger.write(
            frame_id=self._frame_counter,
            tag="battery",
            state="",
            data={"voltage_v": voltage_v, "percent": percent},
        )

    def log_images_lazy(
        self,
        render_fn,
        tags: list,
        state: str = "",
        subdir: Optional[str] = None,
    ) -> Optional[list]:
        """1 回の render_fn 呼び出しで複数タグの画像をまとめてキューに積む．
        render_fn は len(tags) 個の np.ndarray（or None）のリストを返すこと．
        _build_1d_range() 等の重複計算を防ぐバンドル保存向け API．
        """
        if not self._active or not self._config.save_images:
            return None
        return self._image_logger.save_lazy_bundle(
            frame_id=self._frame_counter,
            render_fn=render_fn,
            tags=tags,
            state=state,
            subdir=subdir,
        )

    def log_image_lazy(self, render_fn, tag: str = "", state: str = "", subdir: Optional[str] = None) -> Optional[Path]:
        """描画関数をキューに積む．描画＋保存はワーカースレッドで実行．
        メインループへの影響: ~0.01ms (queue.put のみ)

        Args:
            render_fn:  引数なしで np.ndarray を返す関数．None を返した場合は保存しない．
                        ※ numpy 配列を参照する場合はクロージャで .copy() してキャプチャすること．
        Usage:
            self.log_image_lazy(lambda d=depth.copy(): render_roi(d), tag="roi")
        """
        if not self._active or not self._config.save_images:
            return None
        return self._image_logger.save_lazy(
            frame_id=self._frame_counter,
            render_fn=render_fn,
            tag=tag,
            state=state,
            subdir=subdir,
        )

    def log_image(self, image, tag: str = "", state: str = "", subdir: Optional[str] = None) -> Optional[Path]:
        """画像を保存するヘルパー
        Args:
            image:      保存する画像（numpy配列）
            tag:        画像の種類識別子（例: "aruco_detect", "depth"）
            state:      発生した状態名
            subdir:     画像を保存するサブディレクトリ（例: "aruco", "depth"）。指定しない場合は state 名のサブディレクトリに保存。
        Usage:
            logger.log_image(rgb_image, tag="aruco_detect", state="follow_row", subdir="aruco")
            logger.log_image(depth_color, tag="depth", state="follow_row", subdir="depth")
        """
        if not self._active or not self._config.save_images:
            return None
        return self._image_logger.save(self._frame_counter, image, tag, state, subdir)

    def log_state_transition(self, from_state: str, to_state: str, reason: str = "") -> None:
        """状態遷移履歴を state_transitions.csv に記録する.
        Args:
            from_state: 遷移元状態名
            to_state:   遷移先状態名
            reason:     任意のメモ（トリガ要因など）
        Usage:
            logger.log_state_transition(from_state="idle", to_state="follow_row", reason="marker detected")
        """
        if not self._active or self._state_logger is None:
            return
        self._state_logger.write(
            frame_id=self._frame_counter,
            from_state=from_state,
            to_state=to_state,
            reason=reason,
            t_ros=rospy.Time.now().to_sec(),
        )

    def log_jsonl(self, data: Dict[str, Any], tag: str = "data", state: str = "",) -> Optional[Path]:
        """各状態の任意のデータをJSONとして保存するヘルパー
        Args:            
            data:       保存するデータの辞書
            tag:        データの種類識別子（例: "pose_estimation", "aruco_detection"）
            state:      発生した状態名
        Usage:
            logger.log_jsonl({"x": 1.0, "y": 0.5, "theta": 0.1}, tag="pose_estimation", state="follow_row")
            logger.log_jsonl({"id": 23, "distance": 1.5}, tag="aruco_detection", state="follow_row")
        """
        if not self._active:
            return None
        base = self._session_dir / "states"
        base.mkdir(parents=True, exist_ok=True)

        state_suffix = f"_{state}" if state else "" # 状態名があればタグに追加
        fname = f"{tag}{state_suffix}.jsonl"
        fpath = base / fname

        record = {
            "frame_id": self._frame_counter,
            "timestamp_ros": rospy.get_time() if rospy.core.is_initialized() else None,
            "state": state,
            **data,
        }

        with open(fpath, "a", encoding="utf-8") as f:
            json.dump(record, f, ensure_ascii=False, default=str)
            f.write("\n")

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
                "verbose": self._config.verbose,
            },
        }
        with open(self._session_dir / "session_info.json", "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)