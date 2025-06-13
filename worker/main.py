import threading
import queue
import rospy
import time
import sys
import signal
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
from calibration import calib_turn
from handlers.shutdown_handler import shutdown_event, register_signal_handler

# Initialize ROS node
rospy.init_node('move_charge')
# Publishers and Subscribers
pub     = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
# Message Objects
msg     = OverrideRCIn()
# Set loop rate
rate = rospy.Rate(50)

q = queue.Queue()
camera = CameraHandler()
#illminance = IlluminanceSensor()
imu = IMU()
msg.channels = [REVERSE_VALUE] * 8  # 8チャンネルすべて初期化

register_signal_handler()

def motor():
    """
    モーター制御用の別スレッド
    """
    try:
        while not shutdown_event.is_set():
            try:
                order = q.get(timeout=0.5)
                control.move_motor_func(order, pub)
            except queue.Empty:
                continue
    finally:
        print("[サブ] 終了")


#########################################################

def adjust_theta(theta):
    if theta >= 0:
        if theta < 3:
            return 0
        elif 3 <= theta < 5:
            return theta - (0.7048 * theta + 0.8096)
        else:
            return theta - (0.0002 * theta**2 + -0.0034 * theta + 4.5727)
    elif theta < 0:
        if theta > -4:
            return 0
        else:
            return theta - (0.0018 * theta**2 + 0.1650 * theta + -3.3674)
"""
メインスレッド
"""
def main():
    thread = threading.Thread(target=motor, daemon=False)
    thread.start()

    # 完了フラグ
    turn_completed = False
    #=================
    try:
        rospy.Subscriber('/mavros/imu/data', Imu, imu.imu_callback)

        # IMUデータを取得できるまで待機
        while  not imu.imu_initialized:
            #rospy.loginfo("Waiting for IMU data...")
            rate.sleep()

        state = None
        next_state = None
        pause_sent = False
        theta = None

        while not shutdown_event.is_set():
            # RealSenseカメラデータ取得
            frame, depth, corners, list_ids = camera.get_camera()

            if state == "_pause":
                current_time = time.time()
                if not pause_sent:
                    order.stop(q)
                    pause_sent = True
                    stop_end_time = current_time + PAUSE_TIME
                    continue
                if current_time >= stop_end_time:
                    pause_sent = False      # フラグをリセット
                    state = next_state      # 次の動作状態へ遷移
                continue  # 停止期間内は他の処理を行わない

            if state not in ["_turn_90", "_turn_-90", "_turn_180", "_retry_back"]:
                if state not in ["_arrive_p", "_facing_forward", "_check_position", "_docking_station"]:
                    # 初期状態決定
                    state = transision.decide_state(list_ids)
            print(state)

            if state == "marker10":
                """
                marker10を見つけたら角度調整をしながら近づき,閾値を超えたら停止

                - x_distance: 左右方向のズレ（+は右、-は左）
                - z_distance: 前後方向の距離（奥行き）
                - z_distanceが閾値より大きい場合向きを調整しながら前進
                - 閾値以下の場合は停止
                """
                x, z = camera.camera_calibration(10, MARKER_SIZE_ACTUAL, corners, list_ids)
                if z <= DISTANCE_THRESHOLD_MARKER_10:
                    order.stop(q)
                    next_state = "_turn_180"
                    state = "_pause"
                else:
                    order.adjust_angle(q, x, z)
                        
            elif state == "_turn_90":
                """
                imuデータを取得しながら90°左旋回し停止
                """
                order.turn_theta(90, imu, q)
                order.stop(q)
                next_state = None
                state = "_pause"

            elif state == "_turn_180":
                """
                imuデータを取得しながら180°左旋回し停止
                """
                order.turn_theta(180, imu, q)
                order.stop(q)
                next_state = None
                state = "_pause"
            
            elif state == "marker11":
                """
                marker11を見つけたら角度調整をしながら近づき,閾値を超えたら停止
                """
                x, z = camera.camera_calibration(11, MARKER_SIZE_ACTUAL, corners, list_ids)
                if z <= DISTANCE_THRESHOLD_MARKER_11:
                    order.stop(q)
                    next_state = "_turn_-90"
                    state = "_pause"
                else:
                    order.adjust_angle(q, x, z)
            
            elif state == "_turn_-90":
                """
                imuデータを取得しながら90°右旋回し停止
                """
                order.turn_theta(-90, imu, q)
                order.stop(q)
                next_state = None
                state = "_pause"

            elif state == "_retry_back":
                """
                リトライするための後進機能
                3秒間バックする
                """

            elif state == "charge":
                """
                marker0を見つけたら角度調整をしながら近づき,閾値を超えたら停止
                """
                x, z = camera.camera_calibration(0, MARKER_SIZE_ACTUAL, corners, list_ids)
                if z <= DISTANCE_THRESHOLD_MARKER_0:
                    order.stop(q)
                    next_state = "_arrive_p"
                    state = "_pause"
                else:
                    order.adjust_angle(q, x, z)

            elif state == "_arrive_p":
                """
                目標点Pまで移動
                - 目標点Pを描画,thetaの算出  ターン完了フラグ(turn_completed)をTrueに
                - z_distanceが閾値より大きい場合前進, 閾値以下になったら停止
                ＜例外＞
                - 途中でマーカーを見失った（turn_thetaの角度が大きい）場合, theta*(-1)旋回しバックする
                """
                if 3 in list_ids and 4 in list_ids and not turn_completed:
                    # 目標点Pを描画
                    x3, z3 = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                    x4, z4 = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                    point_a, point_b, point_m, point_p = draw_p.find_P(x3, z3, x4, z4, DISTANCE_POINT_P)
                    theta = draw_p.grayscale_image(point_a, point_b, point_m, point_p) *  (-1)
                    print(f"draw_theta: {theta}")
                    order.turn_theta_diff(adjust_theta(theta), theta, imu, q)
                    order.nudge_stop(q)
                    print(imu.get_yaw())
                    print("___ complete turn theta ___")
                    turn_completed = True                    
                elif (3 in list_ids or 4 in list_ids) and turn_completed:
                    # thetaだけ回転後，目標点まで前進
                    if 3 in list_ids:
                        _, marker_distance = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                    elif 4 in list_ids:
                        _, marker_distance = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                    print(f"distance: {marker_distance}")
                    if marker_distance <= DISTANCE_POINT_MARKER:
                        order.stop(q)
                        print("___ complete arrive at point P ___")
                        next_state = "_facing_forward"
                        state = "_pause"
                        turn_completed  = False
                    else:
                        order.slow_move_forward(q)
                elif not (3 in list_ids or 4 in list_ids):
                    # 途中でマーカーを見失った（turn_thetaの角度が大きい）
                    print("----- lost  3 and 4 markers -----")
                    order.nudge_stop(q)
                    order.turn_theta_diff(adjust_theta(theta*(-1)), theta*(-1), imu, q)
                    order.nudge_stop(q)
                    order.retry_back(q)
                    next_state = None
                    state = "_pause"
                else:
                    print("--------------------------------------a")

            elif state == "_facing_forward":
                """
                マーカー0を基準にステーションに対して正面を向く

                - マーカー0のx_distanceが閾値より大きい場合thetaと逆方向に旋回
                - x_distanceが閾値より小さい場合停止
                - マーカー0を捕捉できないときは捕捉するまでthetaと逆方向に旋回
                """

                if 0 in list_ids:
                    if 3 in list_ids:
                        x3, _ = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                    if 4 in list_ids:
                        x4, _ = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                    x0, _ = camera.camera_calibration(0, MARKER_SIZE_ACTUAL, corners, list_ids)
                    print(f"x0:{x0}")
                    if -3 <= x0 <= 3:
                        order.stop(q)
                        print("___ complete facing forward ___")
                        next_state = "_check_position"
                        state = "_pause"
                        continue
                if 3 in list_ids and 4 in list_ids:
                    x3, _ = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                    x4, _ = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                    #print(f"x3: {x3}, x4: {x4}")
                    if abs(x3) < abs(x4):
                        order.nudge_turn_right(q)
                    else:
                        order.nudge_turn_left(q)
                elif 3 in list_ids:
                    print("Only marker 3 detected → turn right")
                    order.nudge_turn_right(q)
                    order.nudge_stop(q)
                elif 4 in list_ids:
                    print("Only marker 4 detected → turn left")
                    order.nudge_turn_left(q)
                    order.nudge_stop(q)
                # else:
                #     print("Neither marker 3 nor 4 detected → fallback")
                #     order.stop(q)
                #     next_state = None  # もしくは探索やabort処理など
                #     state = "_pause"
            
            elif state == "_check_position":
                """
                ステーションに対して真っすぐ正しい位置にいるか確認

                マーカー（ID: 0, 3, 4）を用いて以下の条件を満たすかをチェックする：
                - 左右に配置されたマーカー3と4のz距離がほぼ等しい（ステーションに対して正対している）
                - 正面にあるマーカー0のx距離がほぼ0（中央を向いている）

                条件を満たしていれば `_docking_station` ステートに進み
                満たさない場合は後退（リトライ）する
                """
                if 3 in list_ids and 4 in list_ids and 0 in list_ids:
                    x3, z3 = camera.camera_calibration(3, MARKER_SIZE_ACTUAL, corners, list_ids)
                    x4, z4 = camera.camera_calibration(4, MARKER_SIZE_ACTUAL, corners, list_ids)
                    x0, z0 = camera.camera_calibration(0, MARKER_SIZE_ACTUAL, corners, list_ids)
                    #marker3,4のzが同じかつ，maker0のxが０
                    # if True:
                    #     print(f"x0: {x0}, z0: {z0}")
                    #     print(f"x3: {x3}, z3: {z3}")
                    #     print(f"x4: {x4}, z4: {z4}")
                    #     print("___ complete facing forward ___")
                    #     next_state = "_docking_station"
                    #     state = "_pause"
                    #     continue
                    if abs(z3-z4) < 3 and abs(x0) < 1.5:
                        print("___ complete facing forward ___")
                        next_state = "_docking_station"
                        state = "_pause"
                        continue
                    else:
                        print("----- back because not posision -----")
                        if abs(z3-z4) < 3:
                            print("not z")
                            # ステーションに真っすぐではあるが，真ん中じゃない
                        elif abs(x0) < 1.5:
                            print("not x")
                            # マーカー０を向いているが真っすぐじゃない
                        order.nudge_stop(q)
                        order.retry_back(q)
                        next_state = None
                        state = "_pause"
                else:
                    order.nudge_stop(q)
                    order.retry_back(q)
                    next_state = None
                    state = "_pause"

            elif state == "_docking_station":
                """
                ステーションにドッキング，充電を行う
                マーカー ID:7 を検出し、そのz距離が 閾値以下であれば接近を停止し、照度センサーで充電が開始されたかを確認

                - 充電が検知されれば `sys.exit()` によりプログラムを正常終了
                - 充電が検知できない場合もログを出して終了する
                - まだ距離が足りない場合はゆっくり前進する
                - マーカー7が見つからなければ一時停止し、再検出を待つ
                """
                if 7 in list_ids:
                    _, z7 = camera.camera_calibration(7, MARKER_SIZE_SMALL, corners, list_ids)
                    if z7 <= DISTANCE_MARKER7:
                        order.nudge_stop(q)
                        print("___ complete docking station ___")
                        next_state = None
                        # if illminance.is_above_threshold(threshold=30):
                        #     print("charging OK   OOOOOOOOOOO")
                        #     sys.exit()
                        # else:
                        #     print("Charging complete not confirmed. Continuing operation.")
                        #     sys.exit()
                        sys.exit()
                    else:
                        order.slow_move_forward(q)
                else:
                    order.stop(q)
                    next_state = "_docking_station"
                    state = "_pause"

            elif state is None:
                order.move_forward(q)

    except Exception as e:
        print(e)
    finally:
        order.stop(q)      
        print("[メイン] サブスレッド終了待機中...")
        shutdown_event.set()  # 念のため再セット
        thread.join()
        print("[メイン] 全終了")

if __name__ == '__main__':
    main()