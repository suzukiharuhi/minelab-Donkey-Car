import cv2
import numpy as np
"""
まず列検出（軽量）を実装 → ロボットを中央で安定させられるか確認。
並列に障害物検出（BEVクラスタ）を追加 → 危険ケースを模したテストを行う。
例外ハンドリング（回避動作、停止）を実装。
必要に応じて深層セグメンテーションや動的障害物追跡を追加。
"""
def preprocess_depth(depth_frame: np.ndarray,
                    min_dist: float=0.2, 
                    max_dist: float=3.0, 
                    roi_ratio: float=0.75) -> np.ndarray:
    """深度画像の前処理
    Args:
        - depth_frame: 入力深度画像 (単一チャネル、float32、距離[m]で表現されていること)
        - min_dist: 有効距離の最小値 [m]
        - max_dist: 有効距離の最大値 [m]
        - roi_ratio: 画像中央部の有効領域比率 (0.0 - 1.0)
    Returns:
        - 前処理後の深度画像
    """
    # --- (1) 範囲外をマスク ---
    depth_clipped = np.where(
        (depth_frame > min_dist) & (depth_frame < max_dist),
        depth_frame,
        0
    )

    # # --- (2) ROI設定（下部カット） ---
    # h, w = depth_clipped.shape
    # roi_end = int(h * roi_ratio)
    # depth_clipped[roi_end:, :] = 0  # 下部を除外（地面・車体など）

    # --- (3) ノイズ除去（メディアンフィルタ） ---
    depth_filtered = cv2.medianBlur(depth_clipped.astype(np.float32), 5)

    # --- (4) 正規化して8bit表示用画像を作成（確認用） ---
    depth_vis = cv2.normalize(depth_filtered, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)

    return depth_filtered, depth_vis

def extract_plant_mask(depth_img: np.ndarray, max_plant_dist: float=1.5) -> np.ndarray:
    """植物領域の二値マスクを作成
    Args:
        - depth_img: 前処理済み深度画像 (float32)
        - max_plant_dist: 植物とみなす距離[m]
    Returns:
        - mask: 0/255の二値マスク
    """
    mask = np.where((depth_img > 0) & (depth_img < max_plant_dist), 255, 0).astype(np.uint8)
    # 小さいノイズ除去
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    # 連結補完
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
    return mask


def detect_crop_rows(mask: np.ndarray, min_width_ratio: float=0.05):
    """列検出：左右列と中央を返す
    Args:
        - mask: 植物領域マスク (uint8)
        - min_width_ratio: 検出列の最小幅（画像幅比）
    Returns:
        - left_col: 左列のx座標
        - right_col: 右列のx座標
        - center: 左右列の中央
        - col_profile: 列方向密度（デバッグ用）
    """
    h, w = mask.shape
    # 縦方向の和（列方向密度）
    column_sum = np.sum(mask, axis=0)
    column_norm = column_sum / np.max(column_sum + 1e-5)  # 0除算防止

    # 閾値で列を抽出
    threshold = 0.3
    col_binary = (column_norm > threshold).astype(np.uint8)

    # 左右列を探す
    contours, _ = cv2.findContours(col_binary.reshape(w,1).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cols = []
    for c in contours:
        x, _, cw, _ = cv2.boundingRect(c)
        if cw > w * min_width_ratio:
            cols.append(x + cw // 2)
    cols = sorted(cols)

    if len(cols) >= 2:
        left_col, right_col = cols[0], cols[-1]
        center = (left_col + right_col) // 2
    else:
        left_col, right_col, center = None, None, w // 2  # デフォルトは画像中央

    return left_col, right_col, center, column_norm


def visualize_rows(depth_img, mask, left_col, right_col, center):
    """列検出結果の可視化"""
    vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    if left_col is not None:
        cv2.line(vis, (left_col, 0), (left_col, vis.shape[0]), (0,255,0), 2)
    if right_col is not None:
        cv2.line(vis, (right_col, 0), (right_col, vis.shape[0]), (0,255,0), 2)
    if center is not None:
        cv2.line(vis, (center, 0), (center, vis.shape[0]), (0,0,255), 2)

    return vis


# -------------------------------
# 使用例
# -------------------------------
if __name__ == "__main__":
    # 深度画像を読み込む (float32, m)
    depth_raw = cv2.imread("depth_example.png", cv2.IMREAD_UNCHANGED)
    depth_m = depth_raw.astype(np.float32) / 1000.0  # mm → m

    # 前処理
    depth_filtered, depth_vis = preprocess_depth(depth_m)

    # 植物マスク作成
    mask = extract_plant_mask(depth_filtered)

    # 列検出
    left, right, center, col_profile = detect_crop_rows(mask)

    # 可視化
    vis = visualize_rows(depth_filtered, mask, left, right, center)

    cv2.imshow("Depth", depth_vis)
    cv2.imshow("Plant Mask", mask)
    cv2.imshow("Row Detection", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

