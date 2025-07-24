## ROSベースの車両制御構成図
<img width="658" alt="Image" src="https://github.com/user-attachments/assets/9fe6dd34-9ef3-4a2a-831b-21b0f497d90e" />

## プロジェクト構成
```
.                                       # プロジェクトルート
├── app/                                # アプリケーション
│   ├── aruco/
│   │   └── aruco_detector.py           # ArUcoマーカー取得クラス
│   ├── subscriber/
│   │   ├── sub_imu.py                  # subscriber(IMUデータ取得用)
│   │   └── sub_realsense.py            # subscriber(realsense取得用)
│   ├── draw_p.py                       # 目標点Pの描画
│   ├── pilot_node.py                   # 車両制御用ノード（アプリケーション）
│   └── state.py                        # 状態管理クラス(IDEL, STOP, BACK, TURN,..)
│
├── config/                             # 定数や状態変数をまとめる
│   └── constant.py                     # 定数 (MARKER_SIZE, カメラ設定など)
│
├── images/                             # 画像保存先ディレクトリ
│
├── ros_pixhawk/                        
│   ├── motor_output.py                 # publisher(モーター出力送信用)
│   └── pixhawk_interface_node.py       # Pixhawkからデータを取得するノード
│
├── mtx.npy                             # realsenseパラメータ
└── dist.npy                            # realsenseパラメータ
```
