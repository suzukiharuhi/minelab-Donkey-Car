import cv2
import os
import numpy as np
from config.constant import *

class GoalPointDrawer:
    def __init__(self, image_size=(640, 480), scale=3):
        self.image_size = image_size
        self.scale = scale # スケーリング係数
        self.center_x = image_size[0] // 2
        self.center_y = image_size[1]

    def convert_coordinates_to_image(self, x, z):
        # 座標を平行移動してスケーリング
        offset_x = self.center_x 
        offset_z = self.center_y
        # スケーリングして座標を変換
        px = int(x * self.scale + offset_x)
        py = int(offset_z - z * self.scale)  # z座標は逆方向にスケーリング

        return px, py
    
    def draw(self, A, B, M, P):
        img = np.full((self.image_size[1], self.image_size[0], 3), (192, 192, 192), dtype=np.uint8)

        A_px, A_py = self.convert_coordinates_to_image(*A)
        B_px, B_py = self.convert_coordinates_to_image(*B)
        M_px, M_py = self.convert_coordinates_to_image(*M)
        P_px, P_py = self.convert_coordinates_to_image(*P)

        # 点の描画
        cv2.circle(img, (A_px, A_py), 5, (255, 0, 0), -1)
        cv2.circle(img, (B_px, B_py), 5, (0, 255, 0), -1)
        cv2.circle(img, (M_px, M_py), 5, (0, 0, 255), -1)
        cv2.circle(img, (P_px, P_py), 5, (0, 255, 255), -1)
        cv2.circle(img, (self.center_x, self.center_y), 5, (0, 0, 0), -1)

        # 線の描画
        cv2.line(img, (self.center_x, self.center_y), (P_px, P_py), (255, 0, 0), 1)

        # 角度計算
        len_x = P_px - self.center_x
        len_y = self.center_y - P_py
        tan_theta = len_x / len_y if len_y != 0 else 0
        theta = np.degrees(np.arctan(tan_theta))

        if len_y < 0 and len_x > 0:
            theta = 180 + theta
        elif len_y < 0 and len_x < 0:
            theta = theta - 180

        # テキスト表示
        cv2.putText(img, f"theta: {theta:.2f}", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        self.capture_image(img)
        print("___ draw goal point P ___")
        return theta
    
    def capture_image(self, frame, save_dir=SAVE_DIR):
        # picture2というディレクトリが存在しない場合は作成
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        # pucture2ディレクトリ内のファイル数を取得
        files = os.listdir(save_dir)
        # ファイル数を取得
        file_number = len(files)
        # ファイル名を作成
        file_name = str(file_number) + ".jpg"
        # ファイルのパスを作成
        file_path = os.path.join(save_dir, file_name)
        # 画像を保存
        cv2.imwrite(file_path, frame)


    @staticmethod
    def find_goal_point(x1, z1, x2, z2, d):
        A = np.array([x1, z1, 0])
        B = np.array([x2, z2, 0])
        M = (A + B) / 2
        AB = B - A
        orthogonal_vector = np.array([-AB[1], AB[0], 0])
        orthogonal_vector = (orthogonal_vector / np.linalg.norm(orthogonal_vector)) * d
        P = M - orthogonal_vector
        return tuple(A[:2]), tuple(B[:2]), tuple(M[:2]), tuple(P[:2])
    