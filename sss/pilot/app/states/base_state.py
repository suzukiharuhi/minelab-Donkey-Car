"""状態の基底クラス"""
from abc import ABC, abstractmethod
from typing import Optional, List
import rospy

from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.logging import SessionLogger

 

class BaseState(ABC):
    """状態の基底クラス"""
    
    def __init__(self):
        self.state_name = self.__class__.__name__ # 状態名(クラス名)取得
        # 一時ロスト管理用カウンタ（全ステート共通で利用可）
        self._miss_count: int = 0
        self._miss_limit_default: int = 5  # デフォルト閾値
    
    def needs_pose_estimation(self) -> bool:
        """
        この状態でマーカーの姿勢推定が必要か（最適化用）
        Returns:
            True: 姿勢・距離が必要
            False: マーカーIDのみでOK（高速）
        """
        return False
    
    @property
    def _logger(self) -> SessionLogger:
        """SessionLoggerのショートカットプロパティ"""
        try:
            return SessionLogger.get_instance()
        except Exception:
            return None
    
    def log_event(self, event_type: str, data: dict) -> None:
        """各状態から手軽にイベントを記録するショートカット"""
        if l := self._logger:
            l.log_event(
                event_type=event_type,
                data=data,
                state=self.__class__.__name__,
            )

    def log_metric(self, metrics: dict, tag: str = "metrics") -> None:
        """各状態から手軽にメトリクスを記録するショートカット"""
        if l := self._logger:
            l.log_metric(
                metrics=metrics,
                state=self.__class__.__name__,
                tag=tag,
            )
    
    def log_image(self, image, tag: str = "image") -> None:
        """各状態から手軽に画像を記録するショートカット"""
        if l := self._logger:
            l.log_image(
                image=image,
                state=self.__class__.__name__,
                tag=tag,
            )

    def log_image_lazy(self, render_fn, tag: str = "image") -> None:
        """描画関数をワーカーへ委譲する．メインループへの負荷ほぼゼロ．
        ※ render_fn 内で numpy 配列を参照する場合は .copy() してデフォルト引数でキャプチャすること．
        例: self.log_image_lazy(lambda d=depth.copy(): render(d), tag="roi")
        """
        if l := self._logger:
            l.log_image_lazy(
                render_fn=render_fn,
                state=self.__class__.__name__,
                tag=tag,
            )

    def log_images_lazy(self, render_fn, tags: list) -> None:
        """1 回の render_fn 呼び出しで複数タグの画像をまとめてキューに積む．
        render_fn は len(tags) 個の np.ndarray（or None）のリストを返すこと．
        _build_1d_range() 等の重複計算防止に使用する．
        例: self.log_images_lazy(lambda: follower.get_debug_images(), ["roi", "heatmap", "gap"])
        """
        if l := self._logger:
            l.log_images_lazy(
                render_fn=render_fn,
                tags=tags,
                state=self.__class__.__name__,
            )
    
    def log_jsonl(self, data: dict, tag: str = "data") -> None:
        """各状態から手軽にJSONデータを記録するショートカット"""
        if l := self._logger:
            l.log_jsonl(
                data=data,
                state=self.__class__.__name__,
                tag=tag,
            )
    
    @abstractmethod
    def enter(self):
        """状態開始時の処理"""
        pass
    
    @abstractmethod
    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """状態処理，制御コマンド生成    
        Args:
            imu_data: IMUデータ
            camera_data: カメラデータ
            markers: 検出されたマーカーリスト
        
        Returns:
            制御コマンド
        """
        pass
    
    @abstractmethod
    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """遷移条件チェック
        Returns:
            次の状態（Noneなら状態維持）
        """
        pass
    
    @abstractmethod
    def exit(self):
        """状態終了時の処理"""
        pass

    def reset_miss_count(self) -> None:
        """ロストカウンタを0にリセット"""
        self._miss_count = 0

    def inc_miss_count(self) -> int:
        """ロストカウンタを1増やして現在値を返す"""
        self._miss_count += 1
        return self._miss_count

    def get_miss_count(self) -> int:
        """現在のロストカウンタを返す"""
        return self._miss_count

    def handle_transient_loss(
        self,
        recover_command: ControlCommand,
        miss_limit: Optional[int] = None,
        log_tag: str = "[BASE_STATE]",
    ) -> ControlCommand:
        """
        一時的な見失いを許容しつつ、超過時は STOP を返す共通ハンドラ（主に execute から使用）。
        - 事前に inc_miss_count() でカウンタを増やしておくこと。
        """
        limit = miss_limit if miss_limit is not None else self._miss_limit_default
        miss = self._miss_count

        if miss <= limit:
            rospy.loginfo(f"{log_tag} marker temporarily lost (miss={miss}/{limit}) → recover",)
            return recover_command

        rospy.loginfo(f"{log_tag} marker lost beyond limit (miss={miss}/{limit}) → STOP")
        return ControlCommand.stop()
    
    def is_lost_beyond_limit(
        self,
        miss_limit: Optional[int] = None,
        log_tag: str = "[BASE_STATE]",
    ) -> bool:
        """
        ロストカウンタが閾値を超えたか判定する（主に check_transition から使用）。
        超過したときに1回だけログを出す想定。
        """
        limit = miss_limit if miss_limit is not None else self._miss_limit_default
        miss = self._miss_count
        if miss > limit:
            rospy.loginfo(f"{log_tag} lost beyond limit (miss={miss}/{limit})")
            return True
        return False