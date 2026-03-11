"""ArUcoマーカー検出"""
import cv2
from cv2 import aruco
import os
import sys
import numpy as np
from typing import List, Optional, Tuple, Dict
import rospy

from pilot.app.utils.data_models import ArUcoMarker, CameraData

class ArucoDetector:
    """ArUcoマーカー検出クラス"""
    
    def __init__(self,  dictionary: int = aruco.DICT_4X4_50, camera_matrix: Optional[np.ndarray] = None, dist_coeffs: Optional[np.ndarray] = None, marker_size: float = 0.07, id_size_map: Optional[Dict[int, float]] = None):
        """
        Args:
            dictionary: ArUco辞書タイプ
            camera_matrix: カメラ行列 (3x3)
            dist_coeffs: 歪み係数
            marker_size: マーカーサイズ（メートル）
        """
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = aruco.DetectorParameters()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size = marker_size
        # IDごとのマーカーサイズ（m）。指定がないIDは self.marker_size を使用
        # 既定: 0/3/4=0.07, 20/21/10=0.15
        self.marker_size_map: Dict[int, float] = id_size_map if id_size_map is not None else {
            0: 0.07, 3: 0.07, 4: 0.07,
            20: 0.15, 21: 0.15, 10: 0.15,
        }
        self.offset_x_cm = 3.0  # カメラ中心からの水平オフセット [cm]

        # ...existing code...
        # サブピクセルで角点精緻化（ピッチ・姿勢の揺れ低減）
        try:
            self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementWinSize = 5
            self.aruco_params.cornerRefinementMaxIterations = 30
            self.aruco_params.cornerRefinementMinAccuracy = 0.01
        except Exception:
            pass

        # rvec/tvec の指数移動平均（小さくするほど平滑・遅延増）
        self._smooth_alpha = 0.2
        self._pose_ema: Dict[int, Dict[str, np.ndarray]] = {}
        # ...existing code...

        if self.camera_matrix is None and self.dist_coeffs is None:
            try:
                # このスクリプトが置いてあるディレクトリ
                base_dir = os.path.dirname(os.path.abspath(__file__))
                # npyファイルの絶対パス
                mtx_path = os.path.join(base_dir, "mtx.npy")
                dist_path = os.path.join(base_dir, "dist.npy")
                self.camera_matrix = np.load(mtx_path)      # intrinsic parameter
                self.dist_coeffs = np.load(dist_path)  # extrinsic parameter
                rospy.loginfo("Camera parameters loaded from mtx.npy and dist.npy")
            except Exception as e:
                rospy.logwarn(f"Failed to load camera parameters: {e}")
                self.camera_matrix = None
                self.dist_coeffs = None
        
        rospy.loginfo(f"ArucoDetector initialized (marker_size: {marker_size}m)")
    
    def detect_markers(self, camera_data: CameraData) -> List[ArUcoMarker]:
        """マーカー検出のみを行う（ID, corners, center）"""
        if camera_data.color_image is None:
            return []

        gray = cv2.cvtColor(camera_data.color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        """
        corners: 二次元配列ではなく、shape=(1, 4, 2)の三次元配列
        例:
            (array([[[171.,  64.],
                [277., 117.],
                [175., 195.],
                [ 72., 116.]]], dtype=float32),)
        ids: 「個々のidが要素数1のリスト」のリスト
        例:
            [[0]]
        """
        
        markers: List[ArUcoMarker] = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # corners[i] は通常 shape (1,4,2) だが、ここでは (4,2) に変換して格納
                corner = corners[i].reshape(-1, 2)
                center = tuple(corner.mean(axis=0).astype(int))
                
                marker = ArUcoMarker(
                    marker_id=int(marker_id),
                    corners=corner,
                    center=center
                )
                markers.append(marker)
        return markers
    
    @staticmethod
    def _rvec_to_euler(rvec: np.ndarray) -> Tuple[float, float, float]:
        """rvec → オイラー角（yaw, pitch, roll）に変換 [deg]

        備考: OpenCV Rodrigues から回転行列Rを得て、Z(=yaw),Y(=pitch),X(=roll) の順で求める。
        """
        R, _ = cv2.Rodrigues(rvec)
        yaw   = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        pitch = float(np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))))
        roll  = float(np.degrees(np.arctan2(R[2, 1], R[2, 2])))
        return yaw, pitch, roll


    def estimate_poses(self, markers: List[ArUcoMarker]) -> List[ArUcoMarker]:
        """        
        与えられた markers に対してまとめて姿勢（rvec,tvec）を付加する
        - rvec, tvec は OpenCVの単位（メートル）
        - distance_x, distance_z は [cm] で格納
        - yaw, pitch, roll も計算して格納
        """
        if not markers:
            return markers
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera parameters missing: pose estimation skipped")
            return markers

        # IDごとのサイズに応じてグルーピングし、それぞれで姿勢推定を行う
        def _size_for_id(mid: int) -> float:
            return self.marker_size_map.get(mid, self.marker_size)

        groups: Dict[float, List[ArUcoMarker]] = {}
        for m in markers:
            sz = _size_for_id(m.marker_id)
            groups.setdefault(sz, []).append(m)

        for size_m, group in groups.items():
            try:
                corners_np = np.array([g.corners for g in group], dtype=np.float32)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners_np,
                    size_m,
                    self.camera_matrix,
                    self.dist_coeffs
                )
            except Exception as e:
                rospy.logwarn(f"estimatePoseSingleMarkers failed (size={size_m}): {e}")
                rvecs, tvecs = None, None

        #     for i, gm in enumerate(group):
        #         r = None
        #         t = None
        #         if rvecs is not None:
        #             r = np.squeeze(np.asarray(rvecs[i]))
        #         if tvecs is not None:
        #             t = np.squeeze(np.asarray(tvecs[i]))

        #         gm.rvec = r
        #         gm.tvec = t

        #         # meters -> cm（t が正しく得られたときのみ計算）
        #         if isinstance(t, np.ndarray) and t.size >= 3:
        #             try:
        #                 x_cm = float(t[0] * 100.0)
        #                 z_cm = float(t[2] * 100.0)
        #                 x_cm -= self.offset_x_cm
        #                 gm.distance_xz = (x_cm, z_cm)
        #             except Exception as e:
        #                 rospy.logwarn(f"tvec conversion error: {e}")

        #         # オイラー角（yaw, pitch, roll）
        #         if isinstance(r, np.ndarray) and r.size >= 3:
        #             try:
        #                 gm.euler_angles = self._rvec_to_euler(r)
        #             except Exception as e:
        #                 rospy.logwarn(f"rvec->Euler conversion error: {e}")
        # return markers
        
            # ...existing code...
            for i, gm in enumerate(group):
                r = None
                t = None
                if rvecs is not None:
                    r = np.squeeze(np.asarray(rvecs[i]))
                if tvecs is not None:
                    t = np.squeeze(np.asarray(tvecs[i]))

                # ---- 時間的平滑化（EMA）----
                if isinstance(r, np.ndarray) and isinstance(t, np.ndarray):
                    prev = self._pose_ema.get(gm.marker_id)
                    if prev is None:
                        r_s, t_s = r, t
                    else:
                        a = self._smooth_alpha
                        r_s = (1 - a) * prev["r"] + a * r
                        t_s = (1 - a) * prev["t"] + a * t
                    self._pose_ema[gm.marker_id] = {"r": r_s, "t": t_s}
                    r, t = r_s, t_s
                # --------------------------------

                gm.rvec = r
                gm.tvec = t

                if isinstance(t, np.ndarray) and t.size >= 3:
                    try:
                        x_cm = float(t[0] * 100.0)
                        z_cm = float(t[2] * 100.0)
                        # 車両中心基準に補正
                        x_cm -= self.offset_x_cm
                        gm.distance_xz = (x_cm, z_cm)
                    except Exception as e:
                        rospy.logwarn(f"tvec conversion error: {e}")

                if isinstance(r, np.ndarray) and r.size >= 3:
                    try:
                        gm.euler_angles = self._rvec_to_euler(r)
                    except Exception as e:
                        rospy.logwarn(f"rvec->Euler conversion error: {e}")
        return markers
        # ...existing code...

    def detect(self, camera_data: CameraData, estimate_pose: bool = False) -> List[ArUcoMarker]:
        """
        互換性のためのラッパー。検出のみ or 検出＋姿勢計算（オプション）
        
        使用例：
        ID のみ必要（高速）: markers = detector.detect(camera_data, estimate_pose=False)
        姿勢も必要: markers = detector.detect(camera_data, estimate_pose=True)
        検出→条件で姿勢: markers = detector.detect_markers(...); if need_pose: detector.estimate_poses(markers)
        """
        markers = self.detect_markers(camera_data)
        if estimate_pose and markers:
            self.estimate_poses(markers)
        return markers

    # ------------- テスト用 -------------- #

    def draw_detected_markers(self, camera_data: CameraData):
        if camera_data.color_image is None:
            return []

        gray = cv2.cvtColor(camera_data.color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )
        aruco.drawDetectedMarkers(camera_data.color_image, corners, ids)
        print(f"corners: {corners}, ids: {ids}")
        save_path = "detected_markers2.jpg"
        cv2.imwrite(save_path, camera_data.color_image)
        os._exit(0)

    def estimate_marker_pose(self, camera_data: CameraData):
        markers = self.detect_markers(camera_data)
        self.estimate_poses(markers)
        for m in markers:
            print(f"Marker ID: {m.marker_id}")
            # print(f"  tvec (m): {m.tvec}")
            # print(f"  rvec (rad): {m.rvec}")
            # print(f"  distance_xz (cm): {m.distance_xz}")
            # print(f"  euler_angles (deg): {m.euler_angles}")
            print(f"   distance_xz: x={m.distance_xz[0]} cm, z={m.distance_xz[1]} cm")
            print(f"   pitch: {m.euler_angles[1]} deg")
            rospy.sleep(1)
        # os._exit(0)

    def save_depth_image(self, camera_data: CameraData):
        # depth_image は float32 → 正規化
        depth_norm = cv2.normalize(camera_data.depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype(np.uint8)

        # カラーマップ（JET = 青→赤）
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        # if camera_data.depth_image is None:
        #     return
        save_path = "depth_image.png"
        # 16ビットPNGとして保存
        cv2.imwrite(save_path, depth_color)
        os._exit(0)
    
    def save_color_image(self, camera_data: CameraData):
        if camera_data.color_image is None:
            return
        save_path = "color_image.jpg"
        cv2.imwrite(save_path, camera_data.color_image)
        os._exit(0)
    


