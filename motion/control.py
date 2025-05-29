import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from config.constant import *
from config.state import *

#def move_motor_func(ord, event, pub, rate):
def move_motor_func(order, pub):
    """
    モータを制御するための関数
    """
    msg = OverrideRCIn()
    msg.channels = [REVERSE_VALUE] * 18

    if order == "move_forward":
        msg.channels[0] = 1440#REVERSE_VALUE 
        msg.channels[2] = ADVANCE_VALUE
        pub.publish(msg)
    elif order == "slow_move_forward":
        msg.channels[0] = REVERSE_VALUE  #1460
        msg.channels[2] = ADVANCE_VALUE 
        pub.publish(msg)
    elif order == "move_backward":
        msg.channels[0] = REVERSE_VALUE 
        msg.channels[2] = BACK_VALUE
        pub.publish(msg)
    elif order == "stop":
        msg.channels[0] = REVERSE_VALUE 
        msg.channels[2] = REVERSE_VALUE 
        pub.publish(msg)
    elif order == "turn_left":
        print("turnleft-----")
        msg.channels[0] = LEFT_TURN
        msg.channels[2] = REVERSE_VALUE 
        pub.publish(msg)
    elif order == "slow_turn_left":
        print("turnleft-----")
        msg.channels[0] = SLOW_LEFT_TURN
        msg.channels[2] = REVERSE_VALUE 
        pub.publish(msg)
    elif order == "turn_right":
        print("turnright-----")
        msg.channels[0] = RIGHT_TURN
        msg.channels[2] = REVERSE_VALUE 
        pub.publish(msg)
    elif order == "slow_turn_right":
        print("turnright-----")
        msg.channels[0] = SLOW_RIGHT_TURN
        msg.channels[2] = REVERSE_VALUE 
        pub.publish(msg)
    
