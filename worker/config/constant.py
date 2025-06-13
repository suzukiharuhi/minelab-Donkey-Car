MARKER_SIZE_ACTUAL = 0.07 # side length of the markers (7 cm)
MARKER_SIZE_SMALL  = 0.03 # side length of the markers (3 cm)
Z_CORRECTION_SLOPE     = 0.9687   # Correction slope for z-axis distance
Z_CORRECTION_INTERCEPT = -0.3535   # Correction intercept for z-axis distance

REVERSE_VALUE        = 1500
ADVANCE_VALUE        = 1900
SLOW_ADVANCE_VALUE   = 1800
BACK_VALUE           = 1100
LEFT_TURN            = 1180#研究室1180
SLOW_LEFT_TURN       = 1200#研究室1200
RIGHT_TURN           = 1830#研究室1830
SLOW_RIGHT_TURN      = 1820#研究室1820


# task names to be added to the queue
STOP           = "stop"
MOVE_FORWARD   = "move_forward"
SLOW_FORWARD   = "slow_move_forward"
MOVE_BACKWARD  = "move_backward"
TURN_LEFT      = "turn_left"
SLOW_TURN_LEFT = "slow_turn_left"
TURN_RIGHT     = "turn_right"
SLOW_TURN_RIGHT= "slow_turn_right"
TURN_180       = "turn_180"

# directory path for saving images 
SAVE_DIR = "/home/pi/catkin_ws/src/worker/image"

# for calibration
#test_angles = [3, 5, 8, 10, 20, 45, 90]
#test_angles = [3, 5, 7, 9]
#test_angles = [-3, -5, -8, -10, -20, -45, -90]
test_angles = [-3, -4, -5, -7]
num_traial = 5


