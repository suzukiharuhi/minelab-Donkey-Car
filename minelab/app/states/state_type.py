"""State-type enumeration (ROS-free)."""
from enum import Enum, auto


class StateType(Enum):
    """Enumeration of all UGV operating states."""

    IDLE = auto()                     # Standby / marker search
    APPROACH_MARKER = auto()          # Approach a detected marker
    CROP_NAVIGATION = auto()          # Navigate between crop rows (FTG-i)
    DOCKING = auto()                  # Dock with the charging station
    AUTO_CHARGING = auto()            # Full automatic charging sequence
    APPROACH_ENTRY_POINT = auto()     # Approach the charging station entry point
    ADJUST_POSITION = auto()          # Align perpendicular to the charging station
    CHARGING = auto()                 # Charging in progress
    RETRYING = auto()                 # Retry docking
    ROTATE180 = auto()                # 180-degree in-place rotation
    WITHDRAWAL = auto()               # Back out of the charging station
    EMERGENCY_STOP = auto()           # Emergency stop on fault detection
