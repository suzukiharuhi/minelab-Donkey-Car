DISTANCE_THRESHOLD_MARKER_0 = 130  # for automatic charging
DISTANCE_THRESHOLD_MARKER_10 = 40
DISTANCE_THRESHOLD_MARKER_11 = 10 

DISTANCE_POINT_P = 60 # (cm)
DISTANCE_POINT_MARKER = 60
DISTANCE_MARKER7 = 10

X_THRESHOLD = 10
X_THRESHOLD_FAR = 40

# Pause time for stopping briefly after completing the moving
PAUSE_TIME = 1.0
REST_TIME  = 1.0


# 少しの角度(3~10°)だけ曲がるときの時間
DURATION_TURN_TIME = 0.1
# リトライでバックする時間
BACK_DURATION = 3

# 指定した角度の旋回　補正
# theta - (0.0002 * theta**2 + -0.0034 * theta + 4.5727) #研究室　10°以上
# theta - (0.7048 * theta + 0.8096)  #研究室　3~10°
# theta - (0.0018 * theta**2 + 0.1650 * theta + -3.3674) #研究室 -3°以下