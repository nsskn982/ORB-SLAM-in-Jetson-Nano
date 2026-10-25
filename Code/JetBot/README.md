# JetBot (jetbot_vslam)

A modular ROS package for **autonomous monocular collision avoidance** on a
**SparkFun JetBot v2.1-based mobile robot** built around the **NVIDIA Jetson Nano
Developer Kit A02**. The A02 exposes a single CSI camera interface, so the entire
perception stack is **monocular**: one IMX219 camera feed is shared between
ORB-SLAM2 localization and obstacle detection, all running onboard.

## Main Function
This package performs **monocular vSLAM with encoder feedback, real-time occupancy
grid mapping, and multi-mode collision avoidance** for autonomous indoor navigation.
Features include:

- **Visual SLAM**: ORB-SLAM2 monocular pose estimation (single CSI camera)
- **Encoder Odometry**: Wheel encoder feedback for robust odometry in low-feature environments
- **Map Persistence**: Save/load SLAM maps for multi-mission autonomy
- **Occupancy Grid**: Real-time **world-axis-aligned** local occupancy grid from the SLAM point cloud
- **Motor Feedback Control**: PID feedback controller to reduce wheel slip and drift
- **Battery Monitoring**: Real-time 2S LiPo voltage and state-of-charge tracking
- **Telemetry Logging**: CSV data recording for post-mission analysis
- **Collision Avoidance**: Map-based (default) or CNN-based detection, or both in parallel
- **Camera Tools**: IMX219 testing, calibration, and data collection utilities

## Problem Description
The JetBot must operate autonomously indoors with sensor fusion, safe collision
detection, power monitoring, and data logging under realistic power and resource
limits (Jetson Nano A02, 4 GB RAM). This package combines monocular ORB-SLAM2 with
encoder fusion, occupancy grid mapping, battery management, and flexible collision
avoidance modes (map-based, CNN, or both). The decision node is **reactive by
default**, with commented goal-seeking scaffolding included for future waypoint
navigation.

## Installation and Setup (Jetson)
1. Install ROS Melodic and catkin tools on Ubuntu 18.04 (JetPack 4.3).
2. From your catkin workspace root, install ROS dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install -y ros-melodic-map-server ros-melodic-orb-slam2-ros
```
3. Install I2C tools and Python dependencies:
```
sudo apt-get install -y python3-smbus i2c-tools
pip3 install -r requirements.txt
```
   - Note: `requirements.txt` includes SparkFun Qwiic drivers (motor, OLED) and Adafruit libraries (battery ADC)
4. Build the workspace and source it:
```
catkin_make
source devel/setup.bash
```
5. Make scripts executable (required for `rosrun`/launch):
```
chmod +x scripts/*.py
```
6. Hardware Setup:
   - Connect wheel encoders to I2C1 on Qwiic pHAT 2.0 (dual-channel counter)
   - Connect battery voltage divider to ADS1115 ADC on I2C1, channel 0
   - Connect motor driver and OLED to I2C1 bus
   - Verify I2C devices: `python3 scripts/qwiic_bus_scan.py --bus 1`
7. **Camera Calibration (REQUIRED before autonomous runs)**:
   - Run the calibration tool using a printed 9x6 chessboard (24 mm squares):
     ```bash
     python3 scripts/imx219_camera_calibrate.py --source csi --sensor-id 0 \
         --board-cols 9 --board-rows 6 --square-size 0.024
     ```
   - Replace the placeholder intrinsics in `config/orb_slam.yaml` with your calibrated output.
     The shipped file contains generic values (`fx=fy=530`, `cx=320`, `cy=240`, zero distortion)
     that will cause ORB-SLAM2 tracking failures and incorrect pose/scale estimates.
     **Do not run autonomously without updating these values.**
8. Place your collision model in [models/](models/) as `collision.onnx` and update the path
   in [launch/system.launch](launch/system.launch) if needed.

## Hardware Test Order (Jetson)
1. Camera live test:
```
python3 scripts/imx219_camera_test.py --source csi --sensor-id 0 --width 1280 --height 720 --fps 30
```
2. Camera calibration (see Setup step 7 — required before SLAM).
3. Qwiic pHAT 2.0 (I2C bus scan):
```
python3 scripts/qwiic_bus_scan.py --bus 1
```
4. Qwiic motor driver test:
```
python3 scripts/qwiic_motor_driver_test.py --speed 120 --duration 1.0
```
5. Qwiic OLED test:
```
python3 scripts/qwiic_oled_test.py --line1 "JetBot" --line2 "OLED OK"
```
6. Launch the ROS system:
```
roslaunch jetbot_vslam system.launch
```

Optional automation:
```
./run_jetbot_all.sh
```

Launch with map_server from the script:
```
./run_jetbot_all.sh --map-yaml /path/to/map.yaml
```

Run CNN in parallel or switch decision node to CNN:
```
./run_jetbot_all.sh --run-cnn
./run_jetbot_all.sh --use-cnn-collision
```

## Code Conventions
- Every ROS node uses a standard `main()` entry point. `rospy.init_node()` is called
  inside `main()` (not in `__init__`), so node classes can be imported/unit-tested
  without spinning up a ROS node.
- All Python files carry **Doxygen** headers (`@file`, `@brief`, `@author`, `@date`)
  and method-level `@brief`/`@param`/`@return` annotations.
- Shared math/helper functions live in `jetbot_utils.py` to avoid duplication.

## Autonomous Features

### Encoder Odometry (`encoder_node.py`)
Publishes encoder-based odometry from wheel ticks for improved localization in low-texture environments.
- **Topic**: `/odom` (Odometry with covariance), `/encoder/twist` (TwistStamped)
- **Parameters**: `wheel_diameter` (0.066 m), `wheelbase` (0.090 m), `ticks_per_revolution` (64), `publish_freq`
- **Enable/Disable**: `--no-encoders` flag in `run_jetbot_all.sh`

### Motor Feedback Control (`motor_feedback_controller.py`)
PID-based velocity feedback controller reduces wheel slip and skid steering drift.
- **Input**: `/cmd_vel` (TwistStamped) + `/jetbot/encoder_ticks`
- **Output**: `/jetbot/motor_commands`, `/motor/joint_state` (diagnostics)
- **Parameters**: `kp`, `ki`, `kd` (PID gains), `max_correction`, `publish_freq`
- **Enable/Disable**: `--no-motor-feedback` flag in `run_jetbot_all.sh`
- Uses the shared `clamp()` from `jetbot_utils.py`.

### Occupancy Grid (`occupancy_grid_node.py`)
Converts ORB-SLAM2 map points into a **robot-centered, WORLD-AXIS-ALIGNED local
occupancy grid** used by the map-based collision detector. Map points are projected
directly using their raw world `dx/dy` offsets from the robot — **no heading
rotation is applied** — so the grid x/y axes stay parallel to the SLAM world frame
regardless of robot yaw. The map origin orientation is identity (`yaw = 0`), which
matches the `origin_yaw = 0` assumption in `map_collision_node._world_to_cell()`.
This world-aligned convention keeps the local grid consistent with a future
global/persistent map.
- **Input**: `/orb_slam2/map_points` (PointCloud2), `/pose` (PoseStamped)
- **Output**: `/map` (OccupancyGrid)
- **Parameters**: `grid_width/height`, `grid_resolution`, `inflation_radius`, `publish_freq`
- **Enable/Disable**: `use_occupancy_grid:=false` arg in `system.launch`

### Battery Monitoring (`battery_monitor_node.py`)
Monitors 2S LiPo battery voltage (6.0V–8.4V) and publishes low/critical alerts.
- **Output**: `/battery/voltage`, `/battery/soc` (state of charge %), `/battery/is_low`, `/battery/is_critical`
- **Hardware**: ADS1115 ADC on I2C1, channel 0 (voltage divider: 2:1 nominal)
- **Parameters**: `adc_channel`, `voltage_scale`, `warning_voltage`, `critical_voltage`, `publish_freq`
- **Enable/Disable**: `--no-battery` flag in `run_jetbot_all.sh`

### Telemetry Logging (`telemetry_logger_node.py`)
Records all telemetry to a CSV file for post-mission analysis.
- **Output**: `~/jetbot_logs/telemetry_YYYYMMDD_HHMMSS.csv`
- **Columns**: timestamp, pose_x/y/z, odom_x/y/theta, cmd_vel_x/z, collision_prob, battery_voltage, battery_soc
- **Enable/Disable**: `--no-telemetry` flag in `run_jetbot_all.sh`

### Map Persistence (`map_persistence_node.py`)
Saves/loads ORB-SLAM2 point clouds for multi-mission autonomy and return-to-base operations.
- **Services**: `save_map`, `load_map`
- **Output**: `~/jetbot_maps/` (compressed .npz + metadata JSON)
- **Parameters**: `map_dir`, `map_name`, `save_interval`, `load_on_startup`
- **Enable/Disable**: `--enable-map-persist` flag in `run_jetbot_all.sh` (default: disabled)

### Shared Utilities (`jetbot_utils.py`)
Common math functions eliminating code duplication across nodes.
- **Functions**: `quaternion_to_yaw()`, `normalize_angle()`, `clamp()`, `as_bool()`
- **Used by**: `decision_node`, `map_collision_node`, `occupancy_grid_node`, `motor_feedback_controller`

## Decision Node (Reactive + Goal-Seeking Scaffolding)
`decision_node.py` is the motion controller. It consumes the monocular ORB-SLAM2
pose (`/pose`) and a collision probability (`/collision/probability`) and publishes
rate-limited velocity commands on `/cmd_vel` (TwistStamped).

- **Default behaviour (reactive only)**: drive forward, hold the current heading,
  slow down on moderate collision risk, and perform a spin/back-off recovery on high
  risk or stale pose (timeout fail-safe).
- **Goal-seeking scaffolding**: hooks for waypoint navigation (a `_goal_callback`,
  `_goal_bearing` helper, and a bearing-correction block in `_control_callback`) are
  written but **commented out**, so the reactive baseline is unchanged. Uncomment the
  GOAL-SEEKING SCAFFOLDING blocks in `__init__` and `_control_callback` to enable
  heading-toward-goal control.
- **Input**: `/pose` (PoseStamped), `/collision/probability` (Float32)
- **Output**: `/cmd_vel` (TwistStamped)
- **Parameters**: `linear_speed`, `turn_speed`, `collision_stop_threshold`,
  `collision_slow_threshold`, `control_hz`

## Run Script Options

The `run_jetbot_all.sh` script orchestrates camera/Qwiic tests, optional model
deploy, ROS launch, and autonomous feature configuration via flags:

| Flag | Effect |
|------|--------|
| `--run-cnn` | Run CNN collision node in parallel; **auto-routes `COLLISION_TOPIC` to `/collision/probability_cnn`** |
| `--use-cnn-collision` | Switch the decision node to consume the CNN collision topic |
| `--no-encoders` | Disable encoder odometry node |
| `--no-motor-feedback` | Disable the PID motor feedback controller |
| `--no-telemetry` | Disable CSV telemetry logging |
| `--no-battery` | Disable battery monitor node |
| `--enable-map-persist` | Enable map save/load persistence (off by default) |
| `--collect-data` | Record an image/pose dataset after the mission |
| `--collect-duration <s>` | Duration (seconds) for `--collect-data` |
| `--map-yaml <path>` | Launch `map_server` with the given map YAML |
| `--model-path <path>` | Copy the given ONNX model into `models/collision.onnx` |
| `--monitor` | Run post-launch `rostopic` checks |

**Full Autonomous System** (all features enabled):
```bash
./run_jetbot_all.sh
```

**Basic Map-Only Collision Avoidance** (fast, low CPU):
```bash
./run_jetbot_all.sh --no-encoders --no-motor-feedback --no-telemetry
```

**With Map Persistence** (for multi-mission autonomy):
```bash
./run_jetbot_all.sh --enable-map-persist
```

**CNN Collision Detection** (optional, runs in parallel with map collision):
```bash
./run_jetbot_all.sh --run-cnn
```

**CNN-Only** (disables map-based collision, decision node uses CNN topic):
```bash
./run_jetbot_all.sh --use-cnn-collision --no-encoders
```

**With Data Collection** (records images post-mission):
```bash
./run_jetbot_all.sh --collect-data --collect-duration 120
```

> **Note on collision topic routing**: When `--run-cnn` is passed, the shell script
> automatically routes `COLLISION_TOPIC` to `/collision/probability_cnn`, and the CNN
> node (`collision_node.py`, default output `/collision/probability`) is remapped to
> `/collision/probability_cnn` in `system.launch`. Do not mix `--run-cnn` with a manual
> override back to `/collision/probability`, as the map-based and CNN nodes would then
> compete on the same topic.

## Map-Based Collision Avoidance (Default)
- Uses the occupancy grid on `/map` plus the current pose to estimate collision risk.
- The map-based detector publishes `/collision/probability` and `/collision/risky` for the decision node.
- The occupancy grid is built **world-axis-aligned** from ORB-SLAM2 map points by
  `occupancy_grid_node.py`; `map_collision_node._world_to_cell()` rotates by
  `-origin_yaw` (= 0 for the identity origin), so it is consistent with the
  world-aligned grid.
- Includes a timeout-based fail-safe if map/pose data goes stale.
- Implemented in [scripts/map_collision_node.py](scripts/map_collision_node.py).

## CNN Collision Avoidance Pipeline (Optional)
### 1) Database - Data Selection and Data Processing
- Collect forward-facing camera frames with labels for collision risk (safe vs. risky).
- Keep balanced coverage across lighting, floors, obstacles, and speeds to avoid bias.
- Process data to match inference: resize to 224x224, RGB ordering (if `swap_rb` is true), and normalize using the mean/std configured in [scripts/collision_node.py](scripts/collision_node.py).
- Implementation: [scripts/cnn_database_process.py](scripts/cnn_database_process.py)

### 2) KDD - Data Transformation and Data Mining
- Apply augmentation (brightness, blur, small rotations) while preserving label correctness.
- Mine failure cases by reviewing misclassifications and edge scenes (reflective floors, shadows, narrow passages).
- Track class balance and remove duplicates or near-duplicates to improve generalization.
- Implementation: [scripts/cnn_kdd_transform.py](scripts/cnn_kdd_transform.py)

### 3) Development to Deployment
- Train and validate the CNN offline, then export to ONNX for Jetson inference.
- Validate the ONNX model with OpenCV DNN on a Jetson test image set before deployment.
- Version the model and record the training data snapshot used for that build.
- Implementation: [scripts/cnn_dev_to_deploy.py](scripts/cnn_dev_to_deploy.py)

TensorFlow training and monitoring:
```
python3 scripts/cnn_train_tf.py --data-dir data/processed --epochs 20 --export-onnx
tensorboard --logdir models/tf_training/logs
```

### 4) Deployment of CNN Model
- Place the ONNX model at [models/](models/) as `collision.onnx`, or set `collision_model_path` in [launch/system.launch](launch/system.launch).
- Use `./run_jetbot_all.sh --model-path /path/to/collision.onnx` to copy the model into place.
- Implementation: [scripts/cnn_deploy_model.py](scripts/cnn_deploy_model.py)

### 5) Evaluation, Verification, and Monitoring
- Offline: track accuracy, precision/recall, and confusion matrices for risky vs safe classes.
- On-device: verify topics `/collision/probability_cnn`, `/collision/risky`, and `/cmd_vel` publish expected values.
- Use `./run_jetbot_all.sh --monitor` to run `rostopic` checks after launch.
- Review throttled warnings from [scripts/collision_node.py](scripts/collision_node.py) for image encoding, size, or inference issues.
- Implementations: [scripts/cnn_evaluate.py](scripts/cnn_evaluate.py), [scripts/cnn_pipeline_monitor.py](scripts/cnn_pipeline_monitor.py)

### 6) Data Collection After VSLAM
- Collect images with pose and collision probability once VSLAM is running.
- Use `./run_jetbot_all.sh --collect-data` to record a dataset after tests.
- Implementation: [scripts/cnn_data_collect.py](scripts/cnn_data_collect.py)

## Troubleshooting (JetPack 4.3)
- CSI camera fails to open: ensure `nvarguscamerasrc` is available and no other process is using the camera. Power-cycle the Jetson if the driver is stuck.
- `Failed to open camera source`: verify the IMX219 is connected, the ribbon cable is seated, and the correct `sensor_id` is used. The Jetson Nano A02 has a single CSI port, so use `--sensor-id 0`.
- `No module named cv2`: install Jetson OpenCV packages with `sudo apt-get install python3-opencv`.
- ROS node will not start from launch: make sure scripts are executable (`chmod +x scripts/*.py`) and your workspace is sourced.
- ORB-SLAM2 crashes or cannot find vocab/settings: confirm paths in [launch/orb_slam2_mono.launch](launch/orb_slam2_mono.launch) and [config/orb_slam.yaml](config/orb_slam.yaml).
- **ORB-SLAM2 tracking failures or wrong pose scale**: `config/orb_slam.yaml` ships with placeholder intrinsics. You must replace `fx`, `fy`, `cx`, `cy`, and distortion terms with values from `imx219_camera_calibrate.py`.
- Poor calibration results: capture well-lit, sharp chessboard frames at multiple angles and distances.
- Qwiic devices not detected: confirm `/dev/i2c-1` exists, wiring is secure, and the correct address is used.
- I2C permission errors: add your user to the `i2c` group or run tests with `sudo`.

**Autonomous Feature Troubleshooting**
- Encoders not publishing: verify Qwiic device is detected via `qwiic_bus_scan.py`, check encoder tick connections, confirm `use_encoders:=true` in launch args.
- Motor feedback controller causes oscillation: reduce `kp` (proportional gain) or increase `max_correction` limit.
- Battery monitor shows zero voltage: check ADS1115 I2C address (0x48 default), verify voltage divider wiring on channel 0, confirm Adafruit libraries installed.
- Occupancy grid not updating: verify `/orb_slam2/map_points` is publishing (ORB-SLAM2 node active), check `/pose` topic has fresh data, confirm `use_occupancy_grid:=true`.
- Occupancy grid orientation: the grid is intentionally **world-axis-aligned** (it does not rotate with the robot's heading). Obstacle cells stay fixed in the SLAM world frame as the robot turns — this is expected behaviour and is consistent with `map_collision_node` (`origin_yaw = 0`).
- Telemetry CSV file empty or missing: confirm `enable_telemetry:=true`, check write permissions on `~/jetbot_logs/`, verify the node is publishing to expected topics.
- Map persistence save fails: ensure `~/jetbot_maps/` directory exists and is writable, check disk space, confirm numpy and json modules available.
- Timeout warnings in logs: indicates sensor data is stale (e.g., collision data >1s old); verify topic publishers are active and running at the expected frequency.
- `AttributeError: 'DecisionNode' object has no attribute '_clamp'`: you are running an old version of `decision_node.py`. This package ships the fixed version (2026-06-02) which calls the imported `clamp()` from `jetbot_utils`.
- Memory errors on long runs (4 GB Jetson Nano): reduce telemetry/logging frequency or restart the mission; consider disabling unused features (`--no-telemetry`, `--no-encoders`).

## Folder Structure
```
.
├── CMakeLists.txt
├── package.xml
├── requirements.txt
├── run_jetbot_all.sh
├── config
│   └── orb_slam.yaml          ← MUST be updated with calibrated intrinsics
├── launch
│   ├── imx219_tools.launch
│   ├── map_server.launch
│   ├── orb_slam2_mono.launch
│   └── system.launch
├── models
│   └── README.md              ← place collision.onnx here
└── scripts
    ├── battery_monitor_node.py
    ├── collision_node.py
    ├── cnn_data_collect.py
    ├── cnn_database_process.py
    ├── cnn_deploy_model.py
    ├── cnn_dev_to_deploy.py
    ├── cnn_evaluate.py
    ├── cnn_kdd_transform.py
    ├── cnn_pipeline_monitor.py
    ├── cnn_train_tf.py
    ├── decision_node.py         ← reactive + commented goal-seeking scaffolding
    ├── encoder_node.py
    ├── imx219_camera_calibrate.py
    ├── imx219_camera_test.py
    ├── jetbot_utils.py
    ├── map_collision_node.py
    ├── map_persistence_node.py
    ├── motor_feedback_controller.py
    ├── occupancy_grid_node.py   ← WORLD-AXIS-ALIGNED grid + main() entry point
    ├── qwiic_bus_scan.py
    ├── qwiic_motor_driver_test.py
    ├── qwiic_oled_test.py
    └── telemetry_logger_node.py
```

## Pipeline Data Flow
```
/camera/image_raw
    └─> orb_slam2_ros_mono ──> /pose (PoseStamped)
                          └──> /orb_slam2/map_points (PointCloud2)
                                   └─> occupancy_grid_node ──> /map (OccupancyGrid)
                                            └─> map_collision_node ──> /collision/probability (Float32)
                                                                  └──> /collision/risky (Bool)
                                                     └─> decision_node ──> /cmd_vel (TwistStamped)
                                                              └─> motor_feedback_controller ──> /jetbot/motor_commands
encoder_node ──> /odom        battery_monitor_node ──> /battery/*        telemetry_logger_node ──> ~/jetbot_logs/*.csv
```
When `--run-cnn` is active, `collision_node` is remapped to `/collision/probability_cnn`
and the decision node consumes that topic instead of the map-based one.

## Folder Descriptions (Links)
- [config/](config/) - Configuration files for ORB-SLAM2 and camera-related settings.
- [launch/](launch/) - ROS launch files that start SLAM, collision inference, decision logic, and camera tools.
- [models/](models/) - Placeholder for ONNX models and related assets used by collision inference.
- [scripts/](scripts/) - Python ROS nodes and camera utilities.

## File Descriptions (Links)
- [CMakeLists.txt](CMakeLists.txt) - Catkin build configuration for the package.
- [package.xml](package.xml) - ROS package metadata and dependencies.
- [requirements.txt](requirements.txt) - Python package requirements for Jetson.
- [run_jetbot_all.sh](run_jetbot_all.sh) - Orchestration script for camera/Qwiic tests, optional model deploy, ROS launch, and autonomous feature configuration.

**Autonomous Feature Nodes**
- [scripts/battery_monitor_node.py](scripts/battery_monitor_node.py) - Monitors 2S LiPo battery voltage and publishes low/critical alerts. Uses ADS1115 ADC on I2C.
- [scripts/encoder_node.py](scripts/encoder_node.py) - Publishes wheel encoder odometry for improved SLAM in low-feature areas.
- [scripts/jetbot_utils.py](scripts/jetbot_utils.py) - Shared utility functions (`quaternion_to_yaw`, `normalize_angle`, `clamp`, `as_bool`) used by decision, map_collision, occupancy_grid, and motor_feedback nodes.
- [scripts/map_persistence_node.py](scripts/map_persistence_node.py) - Saves/loads ORB-SLAM2 point clouds to enable multi-mission autonomy.
- [scripts/motor_feedback_controller.py](scripts/motor_feedback_controller.py) - PID feedback controller for motor velocity correction.
- [scripts/occupancy_grid_node.py](scripts/occupancy_grid_node.py) - Converts the ORB-SLAM2 point cloud to a **world-axis-aligned** local occupancy grid.
- [scripts/telemetry_logger_node.py](scripts/telemetry_logger_node.py) - Records all telemetry to CSV for post-mission analysis.

**Core VSLAM and Collision Nodes**
- [scripts/decision_node.py](scripts/decision_node.py) - Reactive motion control node; consumes pose and collision probability, publishes velocity commands with timeout detection and recovery behaviour. Includes commented goal-seeking scaffolding.
- [scripts/map_collision_node.py](scripts/map_collision_node.py) - Map-based collision detector using occupancy grid raycasting (world-aligned, `origin_yaw = 0`). Includes timeout-based fail-safe.

**CNN Pipeline**
- [scripts/collision_node.py](scripts/collision_node.py) - CNN-based collision detector using ONNX models for real-time inference. When run via `--run-cnn`, publishes to `/collision/probability_cnn`.
- [scripts/cnn_data_collect.py](scripts/cnn_data_collect.py) - Collects training data (images + pose) after VSLAM is running.
- [scripts/cnn_database_process.py](scripts/cnn_database_process.py) - Processes raw data for training: resizing, normalization, class balancing.
- [scripts/cnn_deploy_model.py](scripts/cnn_deploy_model.py) - Deploys ONNX model to target directory with versioning.
- [scripts/cnn_dev_to_deploy.py](scripts/cnn_dev_to_deploy.py) - Orchestrates the training-to-deployment pipeline.
- [scripts/cnn_evaluate.py](scripts/cnn_evaluate.py) - Evaluates CNN performance: accuracy, precision/recall, confusion matrices.
- [scripts/cnn_kdd_transform.py](scripts/cnn_kdd_transform.py) - Data mining and augmentation: finds edge cases, balances classes.
- [scripts/cnn_pipeline_monitor.py](scripts/cnn_pipeline_monitor.py) - Real-time monitoring of CNN inference topics and diagnostics.
- [scripts/cnn_train_tf.py](scripts/cnn_train_tf.py) - TensorFlow training with export to ONNX for Jetson deployment.

**Camera Utilities**
- [scripts/imx219_camera_calibrate.py](scripts/imx219_camera_calibrate.py) - Captures calibration images and generates the intrinsic camera matrix.
- [scripts/imx219_camera_test.py](scripts/imx219_camera_test.py) - Live camera test via CSI or USB with resolution/FPS configuration.

**Hardware Testing**
- [scripts/qwiic_bus_scan.py](scripts/qwiic_bus_scan.py) - Scans the I2C bus for connected Qwiic devices and reports addresses.
- [scripts/qwiic_motor_driver_test.py](scripts/qwiic_motor_driver_test.py) - Tests the SparkFun motor driver with configurable speed and duration.
- [scripts/qwiic_oled_test.py](scripts/qwiic_oled_test.py) - Tests the OLED display by writing text to two lines.

**Config and Launch**
- [config/orb_slam.yaml](config/orb_slam.yaml) - ORB-SLAM2 mono configuration. **Intrinsics must be replaced with calibrated values.**
- [launch/imx219_tools.launch](launch/imx219_tools.launch) - Launches IMX219 test or calibration utilities.
- [launch/map_server.launch](launch/map_server.launch) - Launches map_server with a YAML map file.
- [launch/orb_slam2_mono.launch](launch/orb_slam2_mono.launch) - Launches the ORB-SLAM2 mono node.
- [launch/system.launch](launch/system.launch) - Main system launch: TF, SLAM, occupancy grid, collision, and decision nodes.

---
**Platform**: SparkFun JetBot v2.1-based mobile robot · NVIDIA Jetson Nano Developer Kit A02 (single CSI port → monocular) · JetPack 4.3 · Ubuntu 18.04 · ROS Melodic · Python 3.6
**Author**: Srikanth Nanda · 2026-06-02
