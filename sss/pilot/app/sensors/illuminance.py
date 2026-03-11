
"""BH1750照度センサーを用いた充電判定ユーティリティ

責務:
- 3秒間の照度観測に基づいて「充電中（暗い）か」を真偽で返す
- センサー初期化・読み取り・GPIO電源制御の簡易ラッパ
"""

import time
from typing import List
import smbus
import RPi.GPIO as GPIO


# I2C アドレス（必要に応じて 0x5C）
BH1750_ADDR = 0x23

# 動作モード（1 lx 分解能・連続測定）
BH1750_CONT_HIRES_MODE = 0x10


class BH1750Sensor:
    """BH1750 センサー読み取りクラス"""

    def __init__(self, bus_num: int = 1, addr: int = BH1750_ADDR):
        self.bus = smbus.SMBus(bus_num)
        self.addr = addr

    def read_lux(self) -> float:
        """現在の照度[lx]を取得"""
        # 測定モード設定
        self.bus.write_byte(self.addr, BH1750_CONT_HIRES_MODE)
        time.sleep(0.18)  # データシート推奨待ち時間

        # 2バイト読み込み（MSB, LSB）
        data = self.bus.read_i2c_block_data(self.addr, BH1750_CONT_HIRES_MODE, 2)
        raw = (data[0] << 8) | data[1]
        # 1.2 係数（一般的換算）
        return raw / 1.2


def is_charging(sensor: BH1750Sensor, threshold_lx: float = 40.0, duration_sec: float = 10.0, sample_rate_hz: float = 3.0) -> bool:
    """充電判定（暗さ判定）

    仕様:
    - duration_sec の間サンプリングし平均照度を算出
    - 平均値が `threshold_lx` 未満なら「暗い→充電中」と判定

    引数:
    - sensor: BH1750Sensor インスタンス
    - threshold_lx: 判定閾値[lx]（暗ければ充電中）
    - duration_sec: 観測継続時間[秒]（既定 3.0）
    - sample_rate_hz: サンプリング周波数[Hz]（既定 5.0）
    """
    if duration_sec <= 0:
        return False

    interval = max(0.01, 1.0 / sample_rate_hz)
    values: List[float] = []
    start = time.time()

    while time.time() - start < duration_sec:
        values.append(sensor.read_lux())
        print(f"Current lux: {values[-1]:.2f}")
        time.sleep(interval)

    if not values:
        return False

    avg_lx = sum(values) / len(values)
    # 「暗い」→充電ステーション内想定
    return avg_lx > threshold_lx


def main():
    """スタンドアロン動作確認用"""
    sensor = BH1750Sensor()
    try:
        charging = is_charging(sensor, threshold_lx=20.0, duration_sec=3.0, sample_rate_hz=5.0)
        print(f"充電ステーション内かどうか: {'はい' if charging else 'いいえ'}")
    except KeyboardInterrupt:
        print("\n終了中…")


if __name__ == "__main__":
    main()
