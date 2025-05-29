import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from config.constant import *
from config.state import *

MIN_OUTPUT = 1500
MAX_OUTPUT = 1900
STEP = 10
acc_threshold = 0.1

#def move_motor_func(ord, event, pub, rate):
def motor_output(order, pub):
    """
    モータを制御するための関数
    """
    msg = OverrideRCIn()
    msg.channels = [REVERSE_VALUE] * 18

    if order == "move_forward":
        msg.channels[0] = 1440#REVERSE_VALUE 
        msg.channels[2] = ADVANCE_VALUE
        pub.publish(msg)
