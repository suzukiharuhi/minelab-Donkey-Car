# Donkey Car

## プロジェクト構成
```
.                                       # プロジェクトルート
├── charge/
│   ├── __init__.py                     
│   └── draw_p.py                       # 目標点Pの描画
│
├── config/                             # 定数や状態変数をまとめる
│   ├── constant.py                     # 定数 (MARKER_SIZE, カメラ設定など)
│   └── state.py                        # 状態変数 (角度, 閾値，モーター出力値など)
│
├── motion/                             # 動作関連
│   ├── control.py                      # オーダーに対するモーター制御関数，turn_theta
│   └── order.py                        # Queueへのorder用関数
│
├── save_img/                           # 画像保存用
│   ├── __init__.py                     
│   └──  caputure.py                    # 画像保存（フレーム，保存先ディレクトリ）
│
├── sensors/                            # センサー関連
│   ├── __init__.py                     
│   ├── camera.py                       # CameraHandler 画角にマーカーあるか判断，マーカーとの距離取得
│   ├── imu.py                          # imu_callback用（yaw取得）
│   ├── realsense.py                    # realsense初期化，深度画像・カメラ画像取得
│   ├── illuminance.py                  # illuminanceSensor初期化，照度チェック
│   ├── mtx.npy                         # realsenseパラメータ
│   └── dist.npy                        # realsenseパラメータ
│
├── state/
│   ├── __init__.py
│   └── transision.py                   # 初期状態決定用
│
├── main.py                             # main
├── mtx.npy                             # realsenseパラメータ
└── dist.npy                            # realsenseパラメータ
```
