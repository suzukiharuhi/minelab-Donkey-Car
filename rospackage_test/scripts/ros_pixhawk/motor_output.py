import rospy
from mavros_msgs.msg import OverrideRCIn

class MotorOutput:
    def __init__(self):
        # Pixhawkにrc入力を送るpublisher
        self.rc_pub  = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.msg = OverrideRCIn()
        self.msg.channels = [1500] * 18

    def diff_command(self):
        pass

    def run_motor(self, order):
        if order == "move_forward":
            self.msg.channels[0] = 1500 
            self.msg.channels[2] = 1900
        elif order == "slow_move_forward":
            self.msg.channels[0] = 1500
            self.msg.channels[2] = 1800 
        elif order == "move_backward":
            self.msg.channels[0] = 1500 
            self.msg.channels[2] = 1200
        elif order == "stop":
            self.msg.channels[0] = 1500 
            self.msg.channels[2] = 1500  
        elif order == "turn_left":
            self.msg.channels[0] = 1180
            self.msg.channels[2] = 1500  
        elif order == "slow_turn_left":
            self.msg.channels[0] = 1200
            self.msg.channels[2] = 1500  
        elif order == "turn_right":
            self.msg.channels[0] = 1820
            self.msg.channels[2] = 1500  
        elif order == "slow_turn_right":
            self.msg.channels[0] = 1800
            self.msg.channels[2] = 1500 

        self.rc_pub.publish(self.msg)
