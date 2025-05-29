import time
import numpy as np
from config.constant import *
from config.state import *
from motion import order

"""
    # リストの中身かえる？
    calib_turn.calibrate_turn(imu, q)

    theta = 180
    order.turn_theta(theta - (-0.0073 * theta + 9.3994), theta, imu, q)
    order.stop(q, REST_TIME)
    time.sleep(1)
    print(f"yaw: {imu.get_yaw()}")

"""
def calculate_delta_yaw(initial_yaw, final_yaw):
    delta = (final_yaw - initial_yaw + 540) % 360 - 180
    return delta

results = []
def calibrate_turn(imu, q):
    for theta in test_angles:
        for traial in range(num_traial):
            print(f"turn {theta} degree--")
            initial_yaw = imu.get_yaw() % 360

            order.turn_theta(theta, imu, q)
            order.stop(q, REST_TIME)
            time.sleep(2)  # Wait for the robot to stabilize

            final_yaw = imu.get_yaw() % 360

            delta_yaw = calculate_delta_yaw(initial_yaw, final_yaw)
            print(f"Target: {theta}, Actual: {delta_yaw}")
            results.append((theta, delta_yaw))

    # 集計
    target_angles = np.array([r[0] for r in results])
    actual_angles = np.array([r[1] for r in results])
    errors = actual_angles - target_angles  # 実際 - 目標 = 誤差

    # 線形回帰 (誤差 = a * theta + b)
    A = np.vstack([target_angles, np.ones_like(target_angles)]).T
    a, b = np.linalg.lstsq(A, errors, rcond=None)[0]


    print("\nCalibration Complete")
    print(f"Correction formula: corrected_theta = theta - ({a:.4f} * theta + {b:.4f})")