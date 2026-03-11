import numpy as np

class EntryPointCalculator:
    """
    充電ステーションエントリポイント計算ユーティリティ
    マーカー3と4の座標から目標点Pと角度θを計算する
    """
    def __init__(self):
        self._camera_offset_z = -20  # カメラから車体中心までのz方向距離(cm)
        self._distance_to_P = 60      # 中点から目標点Pまでの距離(cm)

    def compute_theta(self, A, B):
        """
        A, B: 左右マーカーの座標 [x,z] cm
        car_offset_z: 車体中心がカメラから z方向に離れている距離(cm)
        distance: 中点からPまでの距離(cm)
        """
        car_offset_z = self._camera_offset_z
        distance = self._distance_to_P

        # 中点
        C = (A + B) / 2

        # ABベクトル
        AB = B - A

        # 法線ベクトル（x,z平面）
        n = np.array([AB[1], -AB[0]])  # AB[0]=x, AB[1]=z
        n = (n / np.linalg.norm(n)) * distance

        # 点P
        P = C + n

        # 車体中心の位置
        car_pos = np.array([0, car_offset_z])

        # 自車からPへのベクトル
        vec = P - car_pos

        # 水平角 θ
        theta = np.degrees(np.arctan2(vec[0], vec[1]))
        # print(f"point P: {P}, car_pos: {car_pos}")
        return theta
