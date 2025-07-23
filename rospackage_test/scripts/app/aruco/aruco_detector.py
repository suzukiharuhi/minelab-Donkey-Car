import numpy as np
import os
import cv2
from cv2 import aruco
from config.constant import *

class MarkerDetector:
    """
    カメラ情報と画角内のマーカーとその距離を取得するクラス
    """
    def __init__(self):
        # Initialize the ArUco marker detector
        dictionary      = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters      = aruco.DetectorParameters()
        self.detector   = aruco.ArucoDetector(dictionary, parameters)

        self.count = 0

    # Function to retrieve camera image and depth data
    def get_marker(self, color_frame):
        """
        RGB画像内のarucoマーカーをリストに格納
        corners: マーカーの4隅の座標
        list_ids: マーカーIDのリスト
        """
        # detect aruco marker in the frame
        corners, ids, _ = self.detector.detectMarkers(color_frame)
        if ids is None:
            list_ids = []
            # save_dir = SAVE_DIR
            # # カラー画像はそのまま保存
            # cv2.imwrite(os.path.join(save_dir, f"no_marker{self.count}.png"), color_frame)
            self.count += 1

        else:
            list_ids = np.ravel(ids).tolist()
            #print(F"list_ids: {list_ids}")

        return corners, list_ids
    
    def aruco_distance(self, marker_id, size, corners, list_ids):
        # Load camera calibration parameters
        camera_matrix = np.load("mtx.npy")      # intrinsic parameter
        distortion_coeff = np.load("dist.npy")  # extrinsic parameter
        # Check if any markers were detected
        if list_ids is not None:
            for i, id in enumerate(list_ids):
                if id == marker_id:
                    # Extract the corner points of the specified marker
                    corner = corners[i]
                    # Estimate the position of the marker
                    _, tvec, _ = aruco.estimatePoseSingleMarkers(corner, size, camera_matrix, distortion_coeff)
                    # Extract and convert the translation vector to get x, y and z
                    tvec = np.squeeze(tvec)
                    x = tvec[0]     # horizontal distance from camera center to marker
                    y = tvec[1]     # vertical distance from camera center to marker
                    z = tvec[2]     # perpendicular distance from camera to marker (depth distance)
                    # Apply a calibration correction to z to improve accuracy
                    x = x * 100 # Convert from meters to centimeters
                    z = z * 100 # Convert from meters to centimeters
                    if size == MARKER_SIZE_ACTUAL:
                        z = Z_CORRECTION_SLOPE * z + Z_CORRECTION_INTERCEPT
                    # else:
                    #     z = Z_CORRECTION_SLOPE * z + Z_CORRECTION_INTERCEPT
                    #     pass
                    #print(f"marker{marker_id}  x:{x}, z:{z}")
                    return x, z
        # Return default values if the marker is not detected
        return None, None
    