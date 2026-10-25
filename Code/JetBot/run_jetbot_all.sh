#!/usr/bin/env bash
###############################################################################
# @file    run_jetbot_all.sh
# @brief   One-shot orchestration script for the jetbot_vslam package.
#
# @details
#   Runs the full bring-up sequence for the SparkFun JetBot v2.1-based mobile
#   robot on the NVIDIA Jetson Nano Developer Kit A02 (single CSI port ->
#   monocular vSLAM + collision avoidance):
#
#       1. Optional hardware pre-flight checks (camera test, calibration,
#          Qwiic I2C bus scan, motor + OLED tests).
#       2. Optional CNN model deploy (copy an ONNX model into models/).
#       3. Optional camera publisher (feeds /camera/image_raw into ROS).
#       4. ROS system launch (TF, ORB-SLAM2 mono, occupancy grid, collision,
#          decision, encoders, motor feedback, battery, telemetry, persistence).
#       5. Optional post-launch topic monitor.
#       6. Optional dataset collection after VSLAM is running.
#
#   Every stage is guarded by a flag so you can run the full autonomous stack
#   or any subset of it. Flags map onto the args exposed by system.launch.
#
# @note   CRITICAL: passing --run-cnn auto-routes COLLISION_TOPIC to
#         /collision/probability_cnn so the decision node consumes the CNN
#         detector instead of the map-based one. Do not also override the
#         collision topic back to /collision/probability, or the map-based and
#         CNN nodes will compete on the same topic.
#
# @note   config/orb_slam.yaml ships with PLACEHOLDER intrinsics. You MUST run
#         camera calibration and replace them before any autonomous run, or
#         ORB-SLAM2 tracking/scale will be wrong.
#
# @author Srikanth Nanda
# @date   2026-06-02
###############################################################################

# ---------------------------------------------------------------------------
# Shell safety: exit on error (-e), error on unset vars (-u), and fail a
# pipeline if any stage fails (-o pipefail). Keeps bring-up deterministic.
# ---------------------------------------------------------------------------
set -euo pipefail

# ---------------------------------------------------------------------------
# Resolve paths. SCRIPT_DIR is the package root (this file lives there).
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS="${SCRIPT_DIR}/scripts"
CONFIG="${SCRIPT_DIR}/config"
MODELS="${SCRIPT_DIR}/models"
PKG_NAME="jetbot_vslam"

# ===========================================================================
# DEFAULT CONFIGURATION
# Each variable below corresponds to a pipeline stage or a system.launch arg.
# Override any of them via the command-line flags documented in usage().
# ===========================================================================

# --- Pre-flight hardware checks (off by default; opt in for first bring-up) --
RUN_CAMERA=false            # --camera-test      : live IMX219 preview (display tool, no ROS publish)
RUN_CALIBRATION=false       # --calibrate        : capture chessboard frames -> intrinsics YAML
RUN_QWIIC=false             # --qwiic-test       : I2C bus scan + motor + OLED smoke tests
RUN_MODEL_DEPLOY=false      # set implicitly when --model-path is given

# --- Camera publisher (feeds /camera/image_raw) -----------------------------
# imx219_camera_test.py is a DIAGNOSTIC viewer only; it does not publish to ROS.
# Provide the command that actually publishes the CSI stream onto ROS here
# (e.g. a gscam / jetson_camera / cv_camera node). Leave RUN_CAMERA_PUBLISHER
# false if your camera publisher is started elsewhere (e.g. its own launch).
RUN_CAMERA_PUBLISHER=false  # --start-camera-pub : run CAMERA_PUBLISHER_CMD in the background
CAMERA_PUBLISHER_CMD=""     # --camera-pub-cmd "<cmd>" : shell command that publishes /camera/image_raw

# --- ROS system launch (the main stack) -------------------------------------
RUN_ROS=true                # always true unless you only want pre-flight checks

# --- system.launch feature toggles (defaults mirror the launch file) --------
RUN_CNN=false               # --run-cnn          : also run CNN collision_node (parallel)
USE_MAP_COLLISION=true      # set false by --use-cnn-collision (CNN-only mode)
USE_OCCUPANCY_GRID=true     # world-axis-aligned grid feeding map_collision_node
USE_ENCODERS=true           # --no-encoders      : disable wheel-encoder odometry
USE_MOTOR_FEEDBACK=true     # --no-motor-feedback: disable PID motor feedback controller
ENABLE_BATTERY=true         # --no-battery       : disable battery monitor node
ENABLE_TELEMETRY=true       # --no-telemetry     : disable CSV telemetry logging
ENABLE_MAP_PERSIST=false    # --enable-map-persist: enable map save/load services

# --- Collision topic the decision node listens on ---------------------------
# Default = map-based detector. Auto-switched to the CNN topic when CNN is the
# active collision source (see flag handling + finalisation below).
COLLISION_TOPIC="/collision/probability"

# --- map_server (static map) ------------------------------------------------
USE_MAP_SERVER=false        # set implicitly when --map-yaml is given
MAP_YAML=""                 # --map-yaml <path>  : static occupancy map for map_server

# --- CNN model path ---------------------------------------------------------
MODEL_PATH=""               # --model-path <path>: ONNX model to deploy into models/collision.onnx

# --- Post-launch monitor ----------------------------------------------------
RUN_MONITOR=false           # --monitor          : run rostopic checks after launch
MONITOR_TIMEOUT=5.0         # per-topic wait timeout (seconds)

# --- Dataset collection after VSLAM -----------------------------------------
COLLECT_DATA=false          # --collect-data     : record images+pose+collision after launch
COLLECT_DURATION=0          # --collect-duration <s>: seconds to collect (0 = until Ctrl-C)
COLLECT_OUTPUT="${HOME}/jetbot_data"

# ---------------------------------------------------------------------------
# usage(): print help and exit.
# ---------------------------------------------------------------------------
usage() {
  cat <<EOF
Usage: ./run_jetbot_all.sh [OPTIONS]

Orchestrates hardware checks, optional model deploy, the ROS launch, and
optional monitoring / data collection for the ${PKG_NAME} package.

Pre-flight hardware checks (run before launch, then exit unless --ros given):
  --camera-test            Live IMX219 preview (diagnostic display only).
  --calibrate              Run chessboard calibration -> intrinsics YAML.
  --qwiic-test             I2C bus scan + motor driver + OLED smoke tests.
  --checks-only            Run requested checks and DO NOT launch the ROS stack.

Collision avoidance mode:
  --run-cnn                Run CNN collision_node in PARALLEL with map collision.
                           Auto-routes the decision node to /collision/probability_cnn.
  --use-cnn-collision      CNN-ONLY: disable map-based collision; decision node
                           consumes /collision/probability_cnn.

Feature toggles (all enabled by default):
  --no-encoders            Disable wheel-encoder odometry node.
  --no-motor-feedback      Disable PID motor feedback controller.
  --no-battery             Disable battery monitor node.
  --no-telemetry           Disable CSV telemetry logging.
  --enable-map-persist     Enable map persistence (save/load) services.

Maps and models:
  --map-yaml <path>        Start map_server with this static map YAML.
  --model-path <path>      Deploy this ONNX model to models/collision.onnx before launch.

Camera publisher (feeds /camera/image_raw into ROS):
  --start-camera-pub       Start the camera publisher command in the background.
  --camera-pub-cmd "<cmd>" Shell command that publishes /camera/image_raw
                           (implies --start-camera-pub).

Post-launch helpers:
  --monitor                After launch, run rostopic checks on key topics.
  --collect-data           After launch, record a dataset (images+pose+collision).
  --collect-duration <s>   Seconds to collect (0 = until Ctrl-C). Implies --collect-data.

  -h, --help               Show this help and exit.

Examples:
  ./run_jetbot_all.sh                                   # full autonomous stack
  ./run_jetbot_all.sh --no-encoders --no-motor-feedback --no-telemetry
  ./run_jetbot_all.sh --enable-map-persist
  ./run_jetbot_all.sh --run-cnn
  ./run_jetbot_all.sh --use-cnn-collision --no-encoders
  ./run_jetbot_all.sh --map-yaml /path/to/map.yaml
  ./run_jetbot_all.sh --camera-test --qwiic-test --checks-only
  ./run_jetbot_all.sh --collect-data --collect-duration 120
EOF
  exit "${1:-0}"
}

# ---------------------------------------------------------------------------
# Track whether the user asked for any pre-flight check and whether they want
# to skip the launch (checks-only mode).
# ---------------------------------------------------------------------------
CHECKS_ONLY=false

# ===========================================================================
# ARGUMENT PARSING
# ===========================================================================
while [[ $# -gt 0 ]]; do
  case "$1" in
    # --- pre-flight checks ---
    --camera-test)        RUN_CAMERA=true ;;
    --calibrate)          RUN_CALIBRATION=true ;;
    --qwiic-test)         RUN_QWIIC=true ;;
    --checks-only)        CHECKS_ONLY=true ;;

    # --- collision mode ---
    --run-cnn)
      RUN_CNN=true
      # CNN runs in parallel; route the decision node to the CNN topic.
      COLLISION_TOPIC="/collision/probability_cnn"
      ;;
    --use-cnn-collision)
      # CNN-only: turn off the map-based detector and consume the CNN topic.
      RUN_CNN=true
      USE_MAP_COLLISION=false
      COLLISION_TOPIC="/collision/probability_cnn"
      ;;

    # --- feature toggles ---
    --no-encoders)        USE_ENCODERS=false ;;
    --no-motor-feedback)  USE_MOTOR_FEEDBACK=false ;;
    --no-battery)         ENABLE_BATTERY=false ;;
    --no-telemetry)       ENABLE_TELEMETRY=false ;;
    --enable-map-persist) ENABLE_MAP_PERSIST=true ;;

    # --- maps / models (these flags carry a value) ---
    --map-yaml)
      MAP_YAML="${2:?--map-yaml requires a path}"
      USE_MAP_SERVER=true
      shift
      ;;
    --model-path)
      MODEL_PATH="${2:?--model-path requires a path}"
      RUN_MODEL_DEPLOY=true
      shift
      ;;

    # --- camera publisher ---
    --start-camera-pub)   RUN_CAMERA_PUBLISHER=true ;;
    --camera-pub-cmd)
      CAMERA_PUBLISHER_CMD="${2:?--camera-pub-cmd requires a command}"
      RUN_CAMERA_PUBLISHER=true
      shift
      ;;

    # --- monitor / data collection ---
    --monitor)            RUN_MONITOR=true ;;
    --collect-data)       COLLECT_DATA=true ;;
    --collect-duration)
      COLLECT_DURATION="${2:?--collect-duration requires seconds}"
      COLLECT_DATA=true
      shift
      ;;

    -h|--help)            usage 0 ;;
    *)
      echo "ERROR: unknown option '$1'" >&2
      usage 1
      ;;
  esac
  shift
done

# In checks-only mode we never bring up the ROS stack.
if [[ "${CHECKS_ONLY}" == true ]]; then
  RUN_ROS=false
fi

# ===========================================================================
# HELPER FUNCTIONS
# ===========================================================================

# log(): timestamped status line for readable bring-up output.
log() { echo "[$(date '+%H:%M:%S')] $*"; }

# require_file(): abort early if a required path is missing.
require_file() {
  if [[ ! -e "$1" ]]; then
    echo "ERROR: required path not found: $1" >&2
    exit 1
  fi
}

# Collects PIDs of background helpers (camera publisher, roslaunch) so we can
# shut everything down cleanly on Ctrl-C or normal exit.
BG_PIDS=()

# cleanup(): kill any background helpers we started. Registered on EXIT/INT/TERM.
cleanup() {
  log "Shutting down background processes..."
  for pid in "${BG_PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

# ===========================================================================
# 0. ENVIRONMENT SANITY
# ===========================================================================
log "jetbot_vslam bring-up starting (package root: ${SCRIPT_DIR})"

# Source the catkin workspace if it has not been sourced yet, so roslaunch can
# find the jetbot_vslam package. Adjust the path if your workspace differs.
if ! command -v roslaunch >/dev/null 2>&1; then
  echo "ERROR: roslaunch not found. Source ROS Melodic first:" >&2
  echo "       source /opt/ros/melodic/setup.bash" >&2
  echo "       source ~/catkin_ws/devel/setup.bash" >&2
  exit 1
fi

# Make sure all node scripts are executable (rosrun/launch require the +x bit).
log "Ensuring scripts are executable (chmod +x scripts/*.py)"
chmod +x "${SCRIPTS}"/*.py 2>/dev/null || true

# ===========================================================================
# 1. PRE-FLIGHT HARDWARE CHECKS (optional)
# ===========================================================================

# --- 1a. Live camera test (diagnostic preview; press 'q' to close) ----------
if [[ "${RUN_CAMERA}" == true ]]; then
  log "Camera test: opening live IMX219 preview (CSI sensor 0)..."
  python3 "${SCRIPTS}/imx219_camera_test.py" \
      --source csi --sensor-id 0 --width 1280 --height 720 --fps 30
fi

# --- 1b. Camera calibration (writes intrinsics YAML) ------------------------
if [[ "${RUN_CALIBRATION}" == true ]]; then
  log "Camera calibration: capture a 9x6 chessboard (24 mm squares)..."
  python3 "${SCRIPTS}/imx219_camera_calibrate.py" \
      --source csi --sensor-id 0 \
      --board-cols 9 --board-rows 6 --square-size 0.024 \
      --num-frames 20 \
      --output "${CONFIG}/imx219_calibration.yaml"
  log "Calibration written to ${CONFIG}/imx219_calibration.yaml"
  log "REMINDER: copy fx/fy/cx/cy + distortion into ${CONFIG}/orb_slam.yaml"
fi

# --- 1c. Qwiic I2C checks (bus scan + motor + OLED) -------------------------
if [[ "${RUN_QWIIC}" == true ]]; then
  log "Qwiic: scanning I2C bus 1 for connected devices..."
  python3 "${SCRIPTS}/qwiic_bus_scan.py" --bus 1

  log "Qwiic: motor driver smoke test (speed 120, 1.0 s per motor)..."
  python3 "${SCRIPTS}/qwiic_motor_driver_test.py" --speed 120 --duration 1.0

  log "Qwiic: OLED display test..."
  python3 "${SCRIPTS}/qwiic_oled_test.py" --line1 "JetBot" --line2 "OLED OK"
fi

# If the user only wanted checks, stop here.
if [[ "${RUN_ROS}" == false ]]; then
  log "Checks-only mode: skipping ROS launch. Done."
  exit 0
fi

# ===========================================================================
# 2. CNN MODEL DEPLOY (optional)
# Copies an ONNX model to models/collision.onnx with a versioned manifest.
# ===========================================================================
if [[ "${RUN_MODEL_DEPLOY}" == true ]]; then
  require_file "${MODEL_PATH}"
  log "Deploying CNN model: ${MODEL_PATH} -> ${MODELS}/collision.onnx"
  python3 "${SCRIPTS}/cnn_deploy_model.py" \
      --model "${MODEL_PATH}" \
      --dest "${MODELS}/collision.onnx" \
      --input-width 224 --input-height 224
fi

# Guard: if CNN collision is requested, ensure a model actually exists.
if [[ "${RUN_CNN}" == true && ! -f "${MODELS}/collision.onnx" ]]; then
  echo "ERROR: CNN collision requested but ${MODELS}/collision.onnx is missing." >&2
  echo "       Deploy one first, e.g.: --model-path /path/to/collision.onnx" >&2
  exit 1
fi

# ===========================================================================
# 3. CAMERA PUBLISHER (optional)
# ORB-SLAM2 subscribes to /camera/image_raw. imx219_camera_test.py is a viewer
# only and does NOT publish to ROS, so a real publisher must feed that topic.
# Start it here if requested; otherwise assume it is started elsewhere.
# ===========================================================================
if [[ "${RUN_CAMERA_PUBLISHER}" == true ]]; then
  if [[ -z "${CAMERA_PUBLISHER_CMD}" ]]; then
    echo "ERROR: --start-camera-pub set but no CAMERA_PUBLISHER_CMD provided." >&2
    echo "       Pass one with: --camera-pub-cmd \"<command that publishes /camera/image_raw>\"" >&2
    exit 1
  fi
  log "Starting camera publisher in background: ${CAMERA_PUBLISHER_CMD}"
  bash -c "${CAMERA_PUBLISHER_CMD}" &
  BG_PIDS+=("$!")
  sleep 2  # give the publisher a moment to advertise /camera/image_raw
fi

# ===========================================================================
# 4. ROS SYSTEM LAUNCH
# Pass every feature toggle through to system.launch. Booleans are forwarded
# as roslaunch args (true/false). The decision node's collision topic is set
# from COLLISION_TOPIC (auto-routed for CNN modes above).
# ===========================================================================
log "Configuration summary:"
log "  map_collision=${USE_MAP_COLLISION}  run_cnn=${RUN_CNN}  collision_topic=${COLLISION_TOPIC}"
log "  occupancy_grid=${USE_OCCUPANCY_GRID}  encoders=${USE_ENCODERS}  motor_feedback=${USE_MOTOR_FEEDBACK}"
log "  battery=${ENABLE_BATTERY}  telemetry=${ENABLE_TELEMETRY}  map_persist=${ENABLE_MAP_PERSIST}"
log "  map_server=${USE_MAP_SERVER}  map_yaml='${MAP_YAML}'"

log "Launching ${PKG_NAME} system.launch..."
roslaunch "${PKG_NAME}" system.launch \
    use_map_collision:="${USE_MAP_COLLISION}" \
    run_cnn:="${RUN_CNN}" \
    use_occupancy_grid:="${USE_OCCUPANCY_GRID}" \
    use_encoders:="${USE_ENCODERS}" \
    use_motor_feedback:="${USE_MOTOR_FEEDBACK}" \
    enable_battery_monitor:="${ENABLE_BATTERY}" \
    enable_telemetry:="${ENABLE_TELEMETRY}" \
    enable_map_persistence:="${ENABLE_MAP_PERSIST}" \
    use_map_server:="${USE_MAP_SERVER}" \
    map_yaml:="${MAP_YAML}" \
    collision_topic:="${COLLISION_TOPIC}" \
    collision_model_path:="${MODELS}/collision.onnx" &
BG_PIDS+=("$!")
ROSLAUNCH_PID="$!"

# Give the stack a few seconds to come up before monitoring / collecting.
log "Waiting for the stack to initialise..."
sleep 5

# ===========================================================================
# 5. POST-LAUNCH MONITOR (optional)
# Checks that key topics are publishing. Non-fatal: warns but does not abort.
# ===========================================================================
if [[ "${RUN_MONITOR}" == true ]]; then
  log "Monitoring pipeline topics (timeout ${MONITOR_TIMEOUT}s each)..."
  python3 "${SCRIPTS}/cnn_pipeline_monitor.py" --timeout "${MONITOR_TIMEOUT}" || \
    log "WARN: pipeline monitor reported missing/silent topics."
fi

# ===========================================================================
# 6. DATASET COLLECTION (optional)
# Records images + pose + collision probability while the stack runs. Uses the
# same COLLISION_TOPIC the decision node is consuming so labels stay aligned.
# ===========================================================================
if [[ "${COLLECT_DATA}" == true ]]; then
  log "Collecting dataset to ${COLLECT_OUTPUT} (duration=${COLLECT_DURATION}s, 0=until Ctrl-C)..."
  python3 "${SCRIPTS}/cnn_data_collect.py" \
      --output-dir "${COLLECT_OUTPUT}" \
      --image-topic "/camera/image_raw" \
      --pose-topic "/pose" \
      --collision-topic "${COLLISION_TOPIC}" \
      --save-rate 5.0 \
      --duration "${COLLECT_DURATION}" || \
    log "WARN: data collection ended with a non-zero status."
fi

# ===========================================================================
# 7. WAIT
# Block on roslaunch so the script stays alive (and the EXIT trap keeps the
# background helpers tied to this shell). Ctrl-C triggers clean shutdown.
# ===========================================================================
log "System running. Press Ctrl-C to stop."
wait "${ROSLAUNCH_PID}"
