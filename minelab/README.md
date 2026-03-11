# minelab – Modular UGV Project Layout

Research-friendly, ROS1-compatible implementation of the minelab UGV
system.  Designed to solve the **real-time bottleneck** of the original
single-loop `pilot_node` by decomposing the system into independently
running nodes.

---

## Directory structure

```
minelab/
├── launch/                    # ROS launch files
│   ├── sensing.launch         # Perception nodes (depth + ArUco)
│   ├── navigation.launch      # Pilot + vehicle I/O nodes
│   └── logging.launch         # Logger node
│
├── nodes/                     # ROS nodes (only directory with rospy)
│   ├── depth_feature_node.py  # Depth image → DepthFeatures JSON
│   ├── marker_detection_node.py  # RGB image → MarkerInfo JSON
│   ├── pilot_node.py          # State machine → /control/command
│   ├── vehicle_io_node.py     # /control/command → RC override PWM
│   └── logger_node.py         # JSON topics → disk records
│
├── app/                       # ROS-independent Python (no rospy)
│   ├── core/
│   │   ├── commands.py        # ControlCommand, CommandType, SpeedLevel
│   │   ├── state_machine.py   # Generic StateMachine
│   │   ├── safety_guard.py    # Watchdog – issues STOP on timeout
│   │   └── command_mapper.py  # ControlCommand → (cmd_str, spd_str)
│   ├── states/
│   │   ├── state_type.py      # StateType enum
│   │   ├── base_state.py      # Abstract BaseState
│   │   ├── idle_state.py
│   │   ├── crop_navigation_state.py
│   │   ├── approach_marker_state.py
│   │   ├── docking_state.py
│   │   └── auto_charging_state.py
│   ├── perception/
│   │   ├── depth_feature_extractor.py  # ROI → 1-D → FTG-i
│   │   ├── aruco_detector.py           # OpenCV ArUco wrapper
│   │   ├── roi.py                      # ROI helpers
│   │   └── filters.py                  # EMA filter
│   ├── sensors/               # Thread-safe data-holder stubs
│   │   ├── base_sensor.py
│   │   ├── imu.py
│   │   ├── battery.py
│   │   └── realsense.py
│   ├── actuators/
│   │   ├── rc_override_mapper.py  # ControlCommand → PWM channels
│   │   └── motor_command_publisher.py  # Abstract publisher
│   ├── logging/
│   │   ├── session_logger.py    # JSON-Lines + CSV writer
│   │   ├── image_logger.py      # OpenCV image saver
│   │   └── metric_logger.py     # Lightweight CSV metric writer
│   └── utils/
│       ├── config.py            # load_yaml() helper
│       ├── time_utils.py
│       └── math_utils.py
│
├── interfaces/                # Pure-Python dataclasses (no rospy)
│   ├── data_models.py         # SensorSnapshot, DepthFeatures,
│   │                          #   MarkerInfo, PilotStatus
│   ├── sensor_snapshot.py     # Re-export shortcut
│   ├── feature_models.py      # Re-export shortcut
│   └── vehicle_state.py       # Re-export shortcut
│
├── config/
│   ├── topics.yaml            # Canonical topic-name registry
│   ├── camera.yaml            # Camera / perception parameters
│   ├── pilot.yaml             # Pilot / state-machine parameters
│   └── logging.yaml           # Logger parameters
│
└── scripts/
    └── health_check.sh        # Verify all expected topics are live
```

---

## Architecture

```
/camera/depth/...              /camera/color/...
       │                               │
       ▼                               ▼
depth_feature_node          marker_detection_node
       │ /minelab/depth_features       │ /minelab/marker_info
       └──────────────┬────────────────┘
                      ▼
              pilot_node  ←── /mavros/imu/data
                      │ /control/command
                      ▼
            vehicle_io_node
                      │ /mavros/rc/override
                      ▼
                  Pixhawk / PWM

                 (logger_node subscribes to all minelab/* topics)
```

### Key design principle
> **Only `nodes/` may import `rospy`, message types, Subscribers, or
> Publishers.**  Everything in `app/` and `interfaces/` is pure Python
> and can be unit-tested without a ROS environment.

---

## Running

### 1. Start the perception pipeline

```bash
roslaunch ros_ugv sensing.launch
```

This starts:
- `depth_feature_node` – subscribes to the depth camera and publishes
  `/minelab/depth_features` at camera FPS.
- `marker_detection_node` – subscribes to the colour camera and
  publishes `/minelab/marker_info`.

### 2. Start navigation

```bash
roslaunch ros_ugv navigation.launch
```

This starts:
- `minelab_pilot_node` – runs the state machine at 50 Hz and publishes
  `/control/command`.
- `minelab_vehicle_io_node` – translates commands to RC override PWM at
  50 Hz.

### 3. Start logging (optional)

```bash
roslaunch ros_ugv logging.launch session_tag:=run_001
```

Logs land in `/home/pi/ugv_ws/minelab_logs/<timestamp>_run_001/`.

### 4. Health check

```bash
bash minelab/scripts/health_check.sh
```

---

## Configuration

Edit the YAML files in `minelab/config/` and reload via `rosparam` or
restart the launch files:

| File            | Controls |
|-----------------|----------|
| `topics.yaml`   | Topic names – edit to remap without changing node code |
| `camera.yaml`   | ROI ratios, obstacle threshold, ArUco dictionary |
| `pilot.yaml`    | Loop rate, safety timeouts, steering thresholds |
| `logging.yaml`  | Log directory, CSV / image saving switches |

---

## Extending

### Adding a new state

1. Create `minelab/app/states/my_new_state.py` inheriting `BaseState`.
2. Add an entry to `minelab/app/states/state_type.py`.
3. Register it in `pilot_node.py` → `_build_state_machine()`.

### Switching to a custom ROS message

Replace the JSON-over-String transport in `depth_feature_node.py` and
`pilot_node.py` with proper `.msg` definitions.  The app-layer
dataclasses remain unchanged.
