from enum import Enum

class State(Enum):
    """
    状態を管理するクラス
    - IDEL : アイドル状態（マーカーを認識していない）  ぶれでマーカーを一時的に認識できない場合は？？
    - APPROACH_MARKER_XX : マーカーXXに近づく（閾値まで）
    - 
    - 
    - 
    - 
    """
    IDEL = 0
    
    APPROACH_MARKER_0  = 1
    APPROACH_MARKER_10 = 2

    STOP            = 10
    BACK            = 11
    TURN_90         = 12
    TURN_180        = 13

    CALUCULATE_P    = 20
    TURN_TO_P       = 21
    MOVE_TO_P       = 22
    FACE_FORWARD    = 23
    POSITION_CHECK  = 24
    DOKING_STATION  = 25
    CHARGING        = 26

    RETRY           = 30

