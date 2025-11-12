"""状態の基底クラス"""
from abc import ABC, abstractmethod
from typing import Optional, List
import rospy

from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand

 

class BaseState(ABC):
    """状態の基底クラス"""
    
    def __init__(self):
        self.state_name = self.__class__.__name__ # 状態名(クラス名)取得
    
    def needs_pose_estimation(self) -> bool:
        """
        この状態でマーカーの姿勢推定が必要か（最適化用）
        Returns:
            True: 姿勢・距離が必要
            False: マーカーIDのみでOK（高速）
        """
        return False
    
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