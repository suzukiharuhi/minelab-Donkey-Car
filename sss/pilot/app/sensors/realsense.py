"""RealSenseカメラ管理"""
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from typing import Optional

from . import BaseSensor
from pilot.app.utils.data_models import CameraData


class CameraSensor(BaseSensor):
    """RealSenseカメラクラス"""
    
    def __init__(self, color_topic: str = '/camera/color/image_raw', depth_topic: str = '/camera/depth/image_rect_raw'):
        """
        Args:
            color_topic: カラー画像トピック名
            depth_topic: 深度画像トピック名
        """
        super().__init__()
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self._bridge = CvBridge()
        self._color_sub: Optional[rospy.Subscriber] = None
        self._depth_sub: Optional[rospy.Subscriber] = None
        self._latest_data = CameraData(timestamp=rospy.get_time())
        self._lock = False  # 簡易的な同期

        self.rgb_initialized: bool = False
        self.depth_initialized: bool = False
    
    def start(self) -> None:
        """カメラの開始"""
        if not self._is_active:
            self._color_sub = rospy.Subscriber(self.color_topic, Image, self._color_callback)
            self._depth_sub = rospy.Subscriber(self.depth_topic, Image, self._depth_callback)
            self._is_active = True
            rospy.loginfo(f"Camera Sensor started (color: {self.color_topic}, depth: {self.depth_topic})")
    
    def stop(self) -> None:
        """カメラの停止"""
        if self._is_active:
            if self._color_sub is not None:
                self._color_sub.unregister()
                self._color_sub = None
            if self._depth_sub is not None:
                self._depth_sub.unregister()
                self._depth_sub = None
            self._is_active = False
            self.rgb_initialized = False
            self.depth_initialized = False

            rospy.loginfo("Camera Sensor stopped")
    
    def _color_callback(self, msg: Image) -> None:
        """カラー画像のコールバック
        
        Args:
            msg: 画像メッセージ
        """
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._latest_data.color_image = cv_image
            self._latest_data.timestamp = rospy.get_time()

            # 初回受信で初期化完了とみなす
            if not self.rgb_initialized:
                self.rgb_initialized = True
                rospy.loginfo("RGB camera first message received. Initialization complete.")


            # カラー画像が更新されたタイミングでコールバック通知
            if self._callback is not None:
                try:
                    self._callback(self._latest_data)
                except Exception as e:
                    rospy.logwarn(f"Camera color callback error: {e}")
            
        except CvBridgeError as e:
            rospy.logerr(f"Color image conversion error: {e}")
    
    def _depth_callback(self, msg: Image) -> None:
        """深度画像のコールバック
        
        Args:
            msg: 画像メッセージ
        """
        try:
            depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # depth_image = self._bridge.imgmsg_to_cv2(msg, "16UC1")
            self._latest_data.depth_image = np.array(depth_image, dtype=np.float32)

            # 初回受信で初期化完了とみなす
            if not self.depth_initialized:
                self.depth_initialized = True
                rospy.loginfo("Depth camera first message received. Initialization complete.")

            # 深度画像更新時にもコールバック（必要に応じて）
            if self._callback is not None:
                try:
                    self._callback(self._latest_data)
                except Exception as e:
                    rospy.logwarn(f"Camera depth callback error: {e}")

        except CvBridgeError as e:
            rospy.logerr(f"Depth image conversion error: {e}")
    
    def get_latest_data(self) -> CameraData:
        """最新データの取得
        
        Returns:
            最新のCameraData
        """
        return self._latest_data