import time
import rospy
import queue
from config.constant import *
from config.state import *

_last_command = None

def clear_queue(q):
    while not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            break

# 命令を投入する前
def send_command(q, command):
    global _last_command
    if _last_command != command:
        #clear_queue(q)
        q.put(command)
        _last_command = command
        print(f"order : {command}")
    else:
        pass

def stop(q, rest_time):
    clear_queue(q)
    q.put(STOP)
    stop_time = time.time() + rest_time
    return stop_time

def move_forward(q):
    send_command(q, MOVE_FORWARD)

def slow_move_forward(q):
    q.put(SLOW_FORWARD)

def back(q):
    q.put(MOVE_BACKWARD)

def turn_left(q):
    send_command(q, TURN_LEFT)

def slow_turn_left(q):
    send_command(q, SLOW_TURN_LEFT)

def turn_right(q):
    send_command(q, TURN_RIGHT)

def slow_turn_right(q):
    send_command(q, SLOW_TURN_RIGHT)

def turn_theta_diff(theta, theta_actual, imu, q):
    """
    Function to turn the vehicle until it reaches the specified angle (theta).
    Turns left if theta > 0, right if theta < 0.
    """
    def angle_diff(current, reference):
        diff = (current - reference + 360) % 360
        return diff
    
    yaw_degree = imu.get_yaw()
    initial_yaw_degrees = yaw_degree % 360
    target_yaw_degrees  = (initial_yaw_degrees + theta_actual) % 360  # Target angle after the turn
    print(f"initial_yaw_degrees: {initial_yaw_degrees}, target: {target_yaw_degrees}")
    if abs(theta_actual) <= 3:
        return
    else:
        if theta >= 0:
            turn_left(q)
        elif theta < 0:
            turn_right(q)

    while not rospy.is_shutdown():
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
    stop(q, REST_TIME)
        # if theta > 0:
        #     if target_yaw_degrees >= initial_yaw_degrees and degree >= target_yaw_degrees:
        #         break
        #     elif target_yaw_degrees < initial_yaw_degrees and (degree >= target_yaw_degrees and degree <= initial_yaw_degrees - 10):
        #         break
        # elif theta < 0:
        #     if target_yaw_degrees <= initial_yaw_degrees and degree <= target_yaw_degrees:
        #         break
        #     elif target_yaw_degrees > initial_yaw_degrees and initial_yaw_degrees + 10 < degree < target_yaw_degrees:
        #         
        
def turn_theta(theta, imu, q):
    """
    Function to turn the vehicle until it reaches the specified angle (theta).
    Turns left if theta > 0, right if theta < 0.
    """
    def angle_diff(current, reference):
        diff = (current - reference + 360) % 360
        return diff
    
    yaw_degree = imu.get_yaw()
    initial_yaw_degrees = yaw_degree % 360
    target_yaw_degrees  = (initial_yaw_degrees + theta) % 360  # Target angle after the turn
    print(f"initial_yaw_degrees: {initial_yaw_degrees}, target: {target_yaw_degrees}")
    if theta >= 0:
        turn_left(q)
    elif theta < 0:
        turn_right(q)

    while not rospy.is_shutdown():
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
    
def adjust_angle(q, x_distance, z_distance):
    if z_distance < 150:
        threshold = X_THRESHOLD_FAR
    else:
        threshold = X_THRESHOLD

    if x_distance < -1 * threshold:
        turn_left(q)
    elif x_distance > threshold:
        turn_right(q)
    else: # -〇 < x < 〇
        move_forward(q)