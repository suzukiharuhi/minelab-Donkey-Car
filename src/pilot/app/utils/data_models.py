from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import numpy as np

"""
データモデル定義モジュール
各種センサーデータや車両状態を表すデータクラスを定義する
"""

@dataclass
class IMUData:
    """IMUセンサーデータ"""
    orientation: Tuple[float, float, float, float]  # クォータニオン (x, y, z, w)
    euler_angles: Tuple[float, float, float]  # オイラー角 (roll, pitch, yaw) クォータニオンから変換
    angular_velocity: Tuple[float, float, float] # ジャイロ（角速度）
    linear_acceleration: Tuple[float, float, float] # 加速度（ax, ay, az）
    timestamp: float # サンプリング時刻


@dataclass
class CameraData:
    """カメラデータ"""
    color_image: Optional[np.ndarray] = None # RGC画像
    depth_image: Optional[np.ndarray] = None # Depth画像
    timestamp: float = 0.0 # サンプリング時刻


@dataclass
class ArUcoMarker:
    """ArUcoマーカー情報"""
    marker_id: int
    corners: np.ndarray
    center: Tuple[float, float]
    rvec: Optional[np.ndarray] = None  # 回転ベクトル (Rodrigues)
    tvec: Optional[np.ndarray] = None  # 並進ベクトル
    distance_xz: Optional[Tuple[float, float]] = None # カメラ座標系における距離 (distance_x [cm], distance_z [cm])
    euler_angles: Optional[Tuple[float, float, float]] = None  # (yaw, pitch, roll) in degrees

    def __str__(self) -> str:
        dist_x_str = f"{self.distance_xz[0]:.2f}cm" if self.distance_xz is not None else "N/A"
        dist_z_str = f"{self.distance_xz[1]:.2f}cm" if self.distance_xz is not None else "N/A"
        return f"Marker(ID={self.marker_id}, center={self.center}, distance_x={dist_x_str}, distance_z={dist_z_str})"


@dataclass
class VehicleState:
    """車両の状態情報"""
    current_state: int
    imu_data: Optional[IMUData] = None
    camera_data: Optional[CameraData] = None
    detected_markers: List[ArUcoMarker] = field(default_factory=list)
    target_marker_id: Optional[int] = None

    def get_target_marker(self) -> Optional[ArUcoMarker]:
        """ターゲットマーカーの取得"""
        if self.target_marker_id is None:
            return None
        
        for marker in self.detected_markers:
            if marker.marker_id == self.target_marker_id:
                return marker
        return None
    
    def has_marker(self, marker_id: int) -> bool:
        """特定IDのマーカーが検出されているか"""
        return any(m.marker_id == marker_id for m in self.detected_markers)