# Donkey Car

## プロジェクト構成
```
.                                       # プロジェクトルート
├── sensors/                             # センサー関連
│   ├── __init__.py                     # 初期化
│   ├── camera.py                       # RealSense取得，マーカーとの距離
│   ├── imu.py                          # imu_callback用
│   ├── mtx.npy
│   └── dist.npy
│
├── config/                             # 定数や状態変数をまとめる
│   ├── constant.py                     # 定数 (MARKER_SIZE, カメラ設定など)
│   └── state.py                        # 状態変数 (角度, 閾値, フラグなど)
│
├── motion/                             # 動作関連
│   ├── control.py                      # 動作（前進，後進，左右旋回，ストップ，角度旋回）
│   └── marker_navigation.py            # _find_34(), decide_direction_turn()
│
├── vision/                             
│   ├── __init__.py                     # 初期化
│   ├── caputure.py                     # 画像保存
│   ├── distance.py                     # マーカーとの距離チェック
│   └── grayscale_drawer.py             # グレースケール
│
├── station/
│   ├── __init__.py
│   └── docking.py                      # station_detect
│
├── main.py
├── RealSense.py
├── IlluminanceSensor.py
```
