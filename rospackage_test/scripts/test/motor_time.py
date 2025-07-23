# import RPi.GPIO as GPIO
# import time

# # === GPIOピン定義 ===
# ENCODER_A_PIN = 21  # 紫：A相信号

# # === GPIO設定 ===
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# # === 初期状態を記録（HIGHが通常） ===
# prev_state = GPIO.input(ENCODER_A_PIN)

# print("エンコーダ監視開始（Ctrl+Cで終了）")
# start_time = time.time()
# print(f"A相: {'HIGH' if prev_state else 'LOW'}")

# change = False
# change_time = False

# try:
#     while True:
#         current_state = GPIO.input(ENCODER_A_PIN)
#         before_change = time.time()
#         print(f"A相: {'HIGH' if current_state else 'LOW'}")
        
        
#         if prev_state != current_state:
#             change_time = time.time()
#             edge = "立ち上がり (LOW→HIGH)" if current_state == GPIO.HIGH else "立ち下がり (HIGH→LOW)"
#             print(f"{edge} を検出！時刻: {change_time:.6f} 秒")
#             prev_state = current_state
#         # if abs(before_change - change_time) > 0.01:
#         #     print(f"{edge} を検出！時刻: {before_change:.6f} 秒")


#         prev_state = current_state
#         time.sleep(0.0005)  # CPU使用率抑制（0.5ms）

# except KeyboardInterrupt:
#     print("中断されました。")

# finally:
#     GPIO.cleanup()

import RPi.GPIO as GPIO
import time

# ピン設定
PIN_A = 21
PIN_B = 20

# GPIO初期化
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 状態定義：Gray Code
#    状態: (A << 1) | B
#    0b00 = 0
#    0b01 = 1
#    0b11 = 3
#    0b10 = 2

# 状態遷移テーブル: (前の状態, 現在の状態) → 回転方向
transition_table = {
    (0, 1): +1,
    (1, 3): +1,
    (3, 2): +1,
    (2, 0): +1,
    (0, 2): -1,
    (2, 3): -1,
    (3, 1): -1,
    (1, 0): -1
}

# 初期状態
last_state = (GPIO.input(PIN_A) << 1) | GPIO.input(PIN_B)
direction = "停止"
count = 0

try:
    print("エンコーダ監視開始（Ctrl+Cで終了）")
    while True:
        a = GPIO.input(PIN_A)
        b = GPIO.input(PIN_B)
        current_state = (a << 1) | b

        if current_state != last_state:
            move = transition_table.get((last_state, current_state))

            if move == 1:
                direction = "前進"
                count += 1
            elif move == -1:
                direction = "後進"
                count -= 1
            else:
                direction = "不明"

            print(f"A={a}, B={b}, 方向={direction}, カウント={count}")
            last_state = current_state

        time.sleep(0.0005)  # 調整可能

except KeyboardInterrupt:
    print("終了します")

finally:
    GPIO.cleanup()
