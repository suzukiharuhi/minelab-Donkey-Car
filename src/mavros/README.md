## mavros/ README

低レベル入出力レイヤ。モータ制御/IMU 購読などハードウェア近傍の処理を担当。

### 主なファイル
- `motor_operation_node.py`: 受け取った `ControlCommand` を実機モータ出力に変換
- `motor_output.py`: コマンド→速度レベルのバリデーションと出力
- `subscribers/imu_subscriber.py`: IMU メッセージ購読 → 上位へ `IMUData` 供給
- `subscribers/command_subscriber.py`: 上位のコマンド購読

### 役割分担
- 高位ロジックは `pilot/` に集約
- `mavros/` はデバイス依存の実装・変換・配信に専念

### 連携の概念図
`pilot` (ControlCommand) → `mavros/motor_operation_node.py` → 実機モータ

簡潔な概要のみ。詳細は各ファイル内コメント参照。