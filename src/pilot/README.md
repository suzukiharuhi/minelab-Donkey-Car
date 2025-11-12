## pilot/ README

高位制御レイヤ (センサ集約 → 認識 → 状態遷移 → コマンド生成) のコード群。

### ディレクトリ構成 (抜粋)
```
app/
  core/            # パイロット統括・状態遷移
  sensors/         # IMU / RealSense ラッパ
  perception/      # ArUco 検出
  states/          # 個別状態クラス
  turning/         # 旋回純粋ロジック (TurnController)
  utils/           # データモデル (IMUData, ArUcoMarker 等)
```

### 主な状態 (2025-11-12 現在)
- `IdleState` 初期待機・マーカー探索
- `ApproachMarkerState` ターゲットマーカーへ前進 (xオフセット ±5cm で前進左右分岐)
- `Rotate180State` 180°旋回 (TurnController 利用)
- `FollowRowCenterState` 作物列間走行
- Auto charging 系: `approach_entry_point`, `adjust_position`, `docking` etc.

### 旋回ロジックの分離
`turning/turn_controller.py` は IMU yaw(deg) と目標相対角度から
累積角度・残角度を計算し、left/right, slow/normal, 完了を判定。
State 側は `start()` と `step()` の結果を `ControlCommand` にマッピングするのみ。

### 典型フロー
1. センサ更新 → `pilot` が `IMUData`, `CameraData`, `List[ArUcoMarker]` 取得
2. `state_machine.execute()` が現状態の `execute()` 呼び出し
3. `ControlCommand` をモータ出力へ
4. `check_transition()` で次状態判定

### パラメータ例 (ROS param 名)
- `~turn/padding_deg` 早め停止余裕
- `~turn/slow_threshold_deg` 低速化残角度

### 今後拡張予定
- 汎用 `RotateThetaState` の追加
- 旋回 PID 化 / 過走自己キャリブレーション
- 状態遷移グラフの図解

簡潔な概要のみ。詳細はコード参照。