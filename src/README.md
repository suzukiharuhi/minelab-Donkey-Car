# ros_ugv/src 概要

UGV 制御ノード・アプリケーションコードのルート。

主なディレクトリ:
- `mavros/`: モータ出力や IMU 読み取りなど低レベル I/O を扱うノード/クラス。
- `pilot/`: センサ取得 → 認識 → 状態遷移 → 制御コマンド生成の高レベルロジック。
- `nodes/`: 起動用 ROS ノードスクリプト (`pilot_node.py` など)。

## パッケージ内の責務分離
- センサ/ハードウェア依存は `mavros/` と `pilot/app/sensors/` へ局所化。
- 状態遷移は `pilot/app/core/state_machine.py` と `pilot/app/states/` 各状態クラス。
- 個別動作プリミティブ（例: 旋回ロジック）は `pilot/app/turning/turn_controller.py` として状態から呼び出し。

## 最近の追加
- `turning/turn_controller.py`: 角度ベース旋回の純粋ロジック (IMU yaw 度単位)。
- `states/rotate180.py`: 180°旋回状態。TurnController を利用。
- `states/approach_marker.py`: マーカー横オフセット x>=5 → 前進右折, x<=-5 → 前進左折。閾値内は直進。

## 典型フロー
1. `pilot_node.py` 起動
2. センサ購読 (IMU / RealSense)
3. ArUco 検出 → `IMUData`, `CameraData`, `List[ArUcoMarker]`
4. `state_machine.execute()` で現状態の `execute` → `ControlCommand`
5. コマンドをモーター制御へ出力

## 簡易ビルド/起動
```
roslaunch <your_package> pilot.launch  # 例: ノード起動
```

## 今後の予定例
- 任意角度用汎用 `RotateThetaState`
- 旋回 PID 化 / 過走調整自動キャリブレーション
- README 詳細化 (モジュール間インターフェース図)
