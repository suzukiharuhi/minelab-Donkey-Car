import os
import cv2

def capture_image(frame, save_dir):
    """
    Captures and saves the camera image with a timestamped filename
    """
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
