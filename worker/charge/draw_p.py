import cv2
import numpy as np
from save_img import capture
from config.constant import *

def grayscale_image(a, b, m, p):
    # 画像サイズ
    image_size = (640, 480)

    # 画像の中心 (基準点)
    center_x = image_size[0] // 2
    center_y = image_size[1]

    # スケーリング係数 (1単位 = 5ピクセル)
    scale = 3

    # 平行移動してスケーリングした座標を返す関数
    def convert_coordinates_to_image(x, z, image_size):
        # 座標を平行移動してスケーリング
        offset_x = image_size[0] // 2
        offset_z = image_size[1]
        
        # スケーリングして座標を変換
        px = int(x * scale + offset_x)
        py = int(offset_z - z * scale)  # z座標は逆方向にスケーリング

        return px, py

    # 画像作成（グレースケール）
    img = np.full((image_size[1], image_size[0], 3), (192, 192, 192), dtype=np.uint8)

    # 各座標を画像座標に変換
    A_px, A_py = convert_coordinates_to_image(a[0], a[1], image_size)
    B_px, B_py = convert_coordinates_to_image(b[0], b[1], image_size)
    M_px, M_py = convert_coordinates_to_image(m[0], m[1], image_size)
    P_px, P_py = convert_coordinates_to_image(p[0], p[1], image_size)

    # 各座標を描画
    cv2.circle(img, (A_px, A_py), 5, (255, 0, 0), -1)  # A点（青）
    cv2.circle(img, (B_px, B_py), 5, (0, 255, 0), -1)  # B点（緑）
    cv2.circle(img, (M_px, M_py), 5, (0, 0, 255), -1)  # M点（赤）
    cv2.circle(img, (P_px, P_py), 5, (0, 255, 255), -1)  # P点（黄色）
    cv2.circle(img, (image_size[0] // 2, image_size[1]), 5, (0, 0, 0), -1)  # 自車

    cv2.line(img, (int(image_size[0] // 2), int(image_size[1])), (int(P_px), int(P_py)), (255, 0, 0), 1)

    # Calculate angle theta between the two points
    len_x = P_px - image_size[0] // 2
    len_y = image_size[1] - P_py
    tan_theta = len_x / len_y
    theta = np.degrees(np.arctan(tan_theta))
    # Adjust theta for quadrant
    if len_y < 0 and len_x > 0:
        theta = 180 + theta
    elif len_y < 0 and len_x < 0:
        theta = theta - 180

    cv2.putText(img, f"theta: {theta:.2f}", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    capture.capture_image(img, SAVE_DIR)
    print("___ draw goal point P ___")
    return theta

# Function to find the midpoint and points perpendicular to the line segment AB
def find_P(x1, z1, x2, z2, d):
    # Markers Coordinates
    A = np.array([x1, z1, 0])
    B = np.array([x2, z2, 0])
    # Coordinates of the center of the markersAB
    M = np.add(A, B) / 2
    # Calculate the vector AB and its orthogonal vector
    AB = np.subtract(B, A)
    orthogonal_vector = np.array([-AB[1], AB[0], 0])
    # Normalize and scale the orthogonal vector by distance d
    orthogonal_vector = (orthogonal_vector / np.linalg.norm(orthogonal_vector)) * d
    P = np.subtract(M, orthogonal_vector)

    return tuple(A), tuple(B), tuple(M), tuple(P)