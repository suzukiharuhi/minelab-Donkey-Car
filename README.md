# Donkey Car

## プロジェクト構成
```
.                                       # プロジェクトルート
├── aruco/                              # ArUcoマーカー
│   ├── __init__.py                     # 初期化
│   └── distance.py                     # マーカーの位置推定
│  
├── sensors/                             # センサー関連
│   ├── __init__.py                     # 初期化
│   ├── RealSense.py                    # RealSenseカメラ初期化，RGB・depthの取得スクリプト
│   └── Illuminance.py                  # 照度センサー初期化スクリプト
│
├── config/                             # 定数や状態変数をまとめる
│   ├── __init__.py                     # 初期化
│   ├── constant.py                     # 定数 (MARKER_SIZE, カメラ設定など)
│   └── state.py                        # 状態変数 (角度, 閾値, フラグなど)
│
├── motion/                             # 動作関連
│   ├── __init__.py                     # 初期化
│   ├── control.py                      # 単純動作（前進，後進，左右旋回，ストップ）
│   ├── angle.py                        # 角度旋回
│   └── marker_navigation.py            # _find_34(), decide_direction_turn()
│
├── vision/                             # グレースケール
│   ├── __init__.py                     # 初期化
│   ├── caputure.py
│   └── grayscale_drawer.py
│
├── station/
│   ├── __init__.py
│   └── docking.py                      # station_detect
│
├── main.py
├── mtx.npy
├── dist.npy
```
