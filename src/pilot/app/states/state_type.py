"""状態定義"""
from enum import Enum, auto

class StateType(Enum):
    """状態の列挙型"""
    IDLE = auto()                    # 初期待機，マーカー探索
    APPROACH_MARKER = auto()         # 検知マーカーへの接近

    ROTATE180_MARKER10 = auto()       # マーカー10：180度旋回

    ROTATE90LEFT_MARKER20 = auto()    # マーカー20：90度左旋回
    ROTATE90RIGHT_MARKER30 = auto()   # マーカー30：90度右旋回

    FOLLOW_ROW_CENTER = auto()        # 作物列間走行状態

    AUTO_CHARGING = auto()             # 自動充電動作
    APPROACH_ENTRY_POINT = auto()       # 目標点Pへの接近状態
    ADJUST_POSITION = auto()            # 位置調整状態（ステーションの正面を向く）
    DOCKING = auto()                    # 充電ステーションへのドッキング状態
    CHARGING = auto()                   # 充電状態
    RETRYING = auto()                   # 再試行状態（APPROACH_ENTRY_POINTからやり直す）

    EMERGENCY_STOP = auto()          # 異常検知時の緊急停止