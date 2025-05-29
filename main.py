import threading
import queue
import rospy
import time
import sys
from threading import Event

from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu, MagneticField
from config.constant import *
from config.state import *
from sensors.camera import CameraHandler
from sensors.illuminance import IlluminanceSensor
from sensors.imu import IMU
from motion import control, order
from state import transision
from charge import draw_p

# Initialize ROS node
rospy.init_node('move_charge')

# Publishers and Subscribers
pub     = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

# Message Objects
msg     = OverrideRCIn()
# Set loop rate
rate = rospy.Rate(100)

q = queue.Queue()
camera = CameraHandler()
illminance = IlluminanceSensor()
imu = IMU()
msg.channels = [REVERSE_VALUE] * 8  # 8チャンネルすべて初期化
# イベントオブジェクトの作成（スレッド間の終了通知用）

def motor():
    """
    モーター制御用の別スレッド
    """
    while not rospy.is_shutdown():
        try:
            order = q.get()
            control.move_motor_func(order, pub)
        except queue.Empty:
            continue

#########################################################

"""
メインスレッド
"""

# フラグ
tunr_completed = False
theta = None

def main():

    threading.Thread(target=motor, daemon=True).start()

    rospy.Subscriber('/mavros/imu/data', Imu, imu.imu_callback)
    # Wait until getting imu data
    while  not imu.imu_initialized and not rospy.is_shutdown():
        # rospy.loginfo("Waiting for IMU data...")
        rate.sleep()

    # while not rospy.is_shutdown():
    #     acc = imu.get_acc()
    #     print(f"acc:{acc}")
    #     # motor_output.motor_output(MOVE_FORWARD, pub)
    #     rate.sleep()

    stop_time = 0
    while not rospy.is_shutdown():
        current_time = time.time()
        # RealSenseカメラデータ取得
        frame, depth, corners, list_ids = camera.get_camera()

        # stopする時間だったら他の処理をスキップ
        if current_time <= stop_time:
            continue

        if state not in ["_turn_90", "_turn_-90", "_turn_180"]:
            if state not in ["_arrive_pointp", "_find_marker_0"]:
                # 初期状態決定
                state = transision.decide_state(list_ids)

        if state == "marker10":
            """
            marker10を見つけたら角度調整をしながら近づき，閾値を超えたら停止
            """
            x, z = camera.camera_calibration(10, MARKER_SIZE_ACTUAL, corners, list_ids)
            if z <= DISTANCE_THRESHOLD_MARKER_10:
                stop_time = order.stop(q, REST_TIME)
                state = "_turn_180"
            else:
                order.adjust_angle(q, x, z)
                    
        elif state == "_turn_90":
            """
            imuデータを取得しながら90°左旋回し停止
            """
            order.turn_theta(90, imu, q)
            stop_time = order.stop(q, REST_TIME)
            state = ""

        elif state == "_turn_180":
            """
            imuデータを取得しながら180°左旋回し停止
            """
            order.turn_theta(180, imu, q)
            stop_time = order.stop(q, REST_TIME)
            state = ""
        
        elif state == "marker11":
            """
            marker11を見つけたら角度調整をしながら近づき，閾値を超えたら停止
            """
            x, z = camera.camera_calibration(11, MARKER_SIZE_ACTUAL, corners, list_ids)
            if z <= DISTANCE_THRESHOLD_MARKER_11:
                stop_time = order.stop(q, REST_TIME)
                state = "_turn_-90"
            else:
                order.adjust_angle(q, x, z)
        
        elif state == "_turn_-90":
            """
            imuデータを取得しながら90°右旋回し停止
            """
            order.turn_theta(-90, imu, q)
            stop_time = order.stop(q, REST_TIME)
            state = ""
        
        elif state == "charge":
            """
            marker0を見つけたら角度調整をしながら近づき，閾値を超えたら停止
            """
            x, z = camera.camera_calibration(0, MARKER_SIZE_ACTUAL, corners, list_ids)
            if z <= DISTANCE_THRESHOLD_MARKER_10:
                stop_time = order.stop(q, REST_TIME)
                state = "_draw_p"
            else:
                order.adjust_angle(q, x, z)

        elif state == "_arrive_pointp":
            if 3 in list_ids and 4 in list_ids and not tunr_completed:
                # 目標点Pを描画
                x3, z3 = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                x4, z4 = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                point_a, point_b, point_m, point_p = draw_p.find_P(x3, z3, x4, z4, DISTANCE_POINT_P)
                theta = draw_p.grayscale_image(point_a, point_b, point_m, point_p) *  (-1)
                # thetaだけ回転
                order.turn_theta(theta, imu, q)
                print("___ complete turn theta ___")
                tunr_completed = True
                stop_time = order.stop(q, PAUSE_TIME)
            elif (0 in list_ids or 3 in list_ids or 4 in list_ids) and tunr_completed:
                # thetaだけ回転後，目標点まで前進
                if 3 in list_ids:
                    _, marker_distance = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                elif 4 in list_ids:
                    _, marker_distance = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                if marker_distance <= DISTANCE_POINT_MARKER:
                    print("___ complete arrive at point P ___")
                    stop_time = order.stop(q, REST_TIME)
                    state = "_find_marker_0"
                else:
                    order.slow_move_forward(q)
            else:
                # turn_thetaがおかしい
                print("----- cannot turn theta for arrive at point P -----")
                order.stop(q, REST_TIME)
                sys.exit()

        elif state == "_find_marker_0":
            if 0 in list_ids:
                x0, _ = camera.camera_calibration(0, MARKER_SIZE_ACTUAL, corners, list_ids)
                if -3 <= x0 <= 3:
                    stop_time = order.stop(q, REST_TIME)
                    state = "_check_position"
                    continue
            if theta >= 0:
                order.turn_right(q)
            elif theta < 0:
                order.turn_left(q)
        
        elif state == "_check_position":
            if 3 in list_ids and 4 in list_ids:
                x3, z3 = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                x4, z4 = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                if True:
                    print("OK")
                    state = "_docking_station"
                    continue
                else:
                    order.back(q)
                    state = ""
                    continue
            else:
                order.back(q)
                state = ""
                continue

        elif state == "_docking_station":
            if 7 in list_ids:
                _, z7 = camera.camera_calibration(7, MARKER_SIZE_SMALL, corners, list_ids)
                if z7 <= DISTANCE_MARKER7:
                    print("___ complete docking station ___")
                    # テスト必要
                    order.stop(q, REST_TIME)
                    if illminance.is_above_threshold(threshold=30):
                        print("charging OK   OOOOOOOOOOO")
                        sys.exit()
                    else:
                        print("Charging complete not confirmed. Continuing operation.")
                        sys.exit()
                else:
                    order.slow_move_forward(q)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Ctrl+C was pressed")
    except Exception as e:
        print(e)
    finally:
        order.stop(STOP, REST_TIME)