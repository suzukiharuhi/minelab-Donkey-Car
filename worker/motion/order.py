import time
import rospy
import queue
from config.constant import *
from config.state import *
from handlers.shutdown_handler import shutdown_event

_last_command = None

def clear_queue(q):
    while not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            break

def send_command(q, command):
    global _last_command
    if _last_command != command:
        print(f"order : {command}")
        q.put(command)
        _last_command = command

def stop(q):
    clear_queue(q)
    send_command(q, STOP)

def nudge_stop(q, duration=PAUSE_TIME):
    send_command(q, STOP)
    time.sleep(duration)

def move_forward(q):
    send_command(q, MOVE_FORWARD)

def slow_move_forward(q):
    send_command(q, SLOW_FORWARD)
    #q.put(SLOW_FORWARD)

def retry_back(q, duration=BACK_DURATION):
    send_command(q, MOVE_BACKWARD)
    time.sleep(duration)

def turn_left(q):
    send_command(q, TURN_LEFT)
    #q.put(TURN_LEFT)

def slow_turn_left(q):
    send_command(q, SLOW_TURN_LEFT)
    #q.put(SLOW_TURN_LEFT)

def nudge_turn_left(q, duration=DURATION_TURN_TIME):
    send_command(q, SLOW_TURN_LEFT)
    time.sleep(duration)

def turn_right(q):
    send_command(q, TURN_RIGHT)
    #q.put(TURN_RIGHT)

def slow_turn_right(q):
    send_command(q, SLOW_TURN_RIGHT)
    #q.put(SLOW_TURN_RIGHT)

def nudge_turn_right(q, duration=DURATION_TURN_TIME):
    send_command(q, SLOW_TURN_RIGHT)
    time.sleep(duration)


def angle_diff(current, reference):
    if abs(current - reference) < 0.5:
        return 0
    diff = (current - reference + 360) % 360
    return diff

def turn_theta_diff(theta, theta_actual, imu, q):
    """
    Function to turn the vehicle until it reaches the specified angle (theta).
    Turns left if theta > 0, right if theta < 0.
    """    
    yaw_degree = imu.get_yaw()
    initial_yaw_degrees = yaw_degree % 360
    target_yaw_degrees  = (initial_yaw_degrees + theta_actual) % 360  # Target angle after the turn
    print(f"initial_yaw_degrees: {initial_yaw_degrees}, target: {target_yaw_degrees}")
    #3度以下なら曲がらない
    if abs(theta_actual) <= 3:
        return
    elif theta >= 0:
        turn_left(q)
    elif theta < 0:
        turn_right(q)
    try:
        while not shutdown_event.is_set():
            #print(f"{time.time()}")
            degree = imu.get_yaw()
            
            if theta > 0:
                diff = angle_diff(degree, initial_yaw_degrees)
                if theta - diff < 45:
                    slow_turn_left(q)
                if diff >= theta:
                    break
            elif theta < 0:
                diff = angle_diff(initial_yaw_degrees, degree) 
                if abs(theta) - diff < 45:
                    slow_turn_right(q)
                if diff >= abs(theta):
                    break
    except KeyboardInterrupt:
        stop(q)
        raise

        
def turn_theta(theta, imu, q):
    """
    Function to turn the vehicle until it reaches the specified angle (theta).
    Turns left if theta > 0, right if theta < 0.
    """
    yaw_degree = imu.get_yaw()
    initial_yaw_degrees = yaw_degree % 360
    target_yaw_degrees  = (initial_yaw_degrees + theta) % 360  # Target angle after the turn
    print(f"initial_yaw_degrees: {initial_yaw_degrees}, target: {target_yaw_degrees}")
    if theta >= 0:
        turn_left(q)
    elif theta < 0:
        turn_right(q)
    try:
        while not shutdown_event.is_set():
            #time.sleep(0.05)
            current_yaw = imu.get_yaw()
            if theta > 0:
                diff = angle_diff(current_yaw, initial_yaw_degrees)
                if diff >= theta:
                    print("g")
                    stop(q)
                    return
                elif theta - diff < 45:
                    slow_turn_left(q)
            elif theta < 0:
                diff = angle_diff(initial_yaw_degrees, current_yaw) 
                if diff >= abs(theta):
                    stop(q)
                    return
                elif abs(theta) - diff < 45:
                    slow_turn_right(q)
                
    except KeyboardInterrupt:
        stop(q)
        raise
    
def adjust_angle(q, x_distance, z_distance):
    """
    マーカーの位置（x方向・z方向）に応じて、車両の進行方向を調整する関数

    - z_distanceが150未満の場合は「遠距離」と判断し、許容ズレ幅（threshold）を大きく取る
    - x_distanceがthreshold内に収まっていれば前進、それ以外は左右に回転する
    """
    if z_distance < 150:
        threshold = X_THRESHOLD_FAR
    else:
        threshold = X_THRESHOLD

    if x_distance < -1 * threshold:
        turn_left(q)
    elif x_distance > threshold:
        turn_right(q)
    else: # -〇 < x < 〇
        slow_move_forward(q)