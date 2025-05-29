import pyrealsense2 as rs
import numpy as np
import cv2
from typing import Tuple, Any

class RealSense:
    """
    RealSenseカメラから深度画像とカメラ映像を取得するクラス
    """
    def __init__(self) -> None:
        """
        RealSenseカメラの設定とパイプラインの初期化を行う
        """
        config                  = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.cap                = cv2.VideoCapture(4)
        self.pipeline           = rs.pipeline()
        pipeline_profile        = self.pipeline.start(config)

        # デバイスの内部パラメータを取得
        profile                 = self.pipeline.get_active_profile()
        depth_profile           = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics        = depth_profile.get_intrinsics()
        
        # print("depth_intrinsics")
        # print(depth_intrinsics)

        # --- 映像安定化のために初期化直後に数フレーム捨てる ---
        for _ in range(10):
            self.cap.read()

        # config = rs.config()
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # pipeline = rs.pipeline()
        # pipeline.start(config)

    def get_frame(self) -> Tuple[Any, np.ndarray]:
        """
        深度画像とカメラ映像を取得し、NumPy配列で返す

        Returns:
            Tuple[Any, np.ndarray]: color_frameとdepth_imageのタプル
        """
        frames                  = self.pipeline.wait_for_frames(timeout_ms=5000)
        depth_frame             = frames.get_depth_frame()
        
        # カメラ映像を取得
        #for _ in range(10):
        _, color_frame      = self.cap.read()

        # 深度データをNumPy配列に変換
        depth_image             = np.asanyarray(depth_frame.get_data())

        return color_frame, depth_image