import numpy as np
import os
import cv2
from cv2 import aruco
from .realsense import RealSense
from config import constant, state

class CameraHandler:
    """
    カメラ情報と画角内のマーカーを取得するクラス
    """
    def __init__(self):
        self.camera = RealSense()

        # Initialize the ArUco marker detector
        dictionary      = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters      = aruco.DetectorParameters()
        self.detector   = aruco.ArucoDetector(dictionary, parameters)

    # Function to retrieve camera image and depth data
    def get_camera(self):
        data = self.camera.get_frame()
        if len(data) == 2:
            frame, depth = data
        else:
            return None, None, None, None
        
        # detect aruco marker in the frame
        corners, ids, rejected = self.detector.detectMarkers(frame)
        if ids is None:
            #print("No markers............")
            list_ids = []
        else:
            list_ids = np.ravel(ids).tolist()
            #print(f"get markers.... {list_ids}")

        return frame, depth, corners, list_ids
    
    def camera_calibration(self, marker_id, size, corners, list_ids):
        # Load camera calibration parameters
        camera_matrix = np.load("mtx.npy")      # intrinsic parameter
        distortion_coeff = np.load("dist.npy")  # extrinsic parameter
        # frame, _ = self.camera.get_frame()
        # corners, ids, _ = self.detector.detectMarkers(frame)
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
                    if size == constant.MARKER_SIZE_ACTUAL:
                        z = constant.Z_CORRECTION_SLOPE * z + constant.Z_CORRECTION_INTERCEPT
                    else:
                        #z = Z_CORRECTION_SLOPE * z + Z_CORRECTION_INTERCEPT
                        pass
                    """
                    z:カメラの中心からマーカーの奥行距離
                    x:カメラの中心からマーカーの水平方向距離（x>0：左側, x<0：右側）
                    """
                    #print(f"marker{marker_id}  x:{x}, z:{z}")
                    return x, z
        # Return default values if the marker is not detected
        print(f"Marker {marker_id} not detected.")
        return 40000, 40000
    