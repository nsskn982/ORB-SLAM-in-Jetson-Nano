# Localization of a Mobile Robot

**Author:** Srikanth Nanda  
**Matriculation Number:** 7026002

## Academic Supervision

| Role | Name |
|------|------|
| 1st Examiner | Prof. Dr. Elmar Wings |
| 2nd Examiner | Prof. Dr. Jonas Schwarz |

## Project Overview

This thesis project focuses on implementing and evaluating localization algorithms for a mobile robot platform. The system uses a SparkFun JetBot equipped with an NVIDIA Jetson Nano as the computational core, enhanced with a Waveshare 16578 Wireless-AC8265 WiFi card for improved connectivity and data transmission capabilities.

### Research Focus

**Primary Implementation Areas:**
- **Object Collision Avoidance**: Real-time obstacle detection and avoidance behaviors
- **Monocular Localization**: Single camera-based position estimation and tracking
- **Dataset Development**: Comprehensive dataset collection and annotation for JetBot training

---

## Repository Guide
### Problem description
Short summary: implement and evaluate localization and collision-avoidance algorithms for a SparkFun JetBot using an NVIDIA Jetson Nano. The repository contains source code, datasets, experiment scripts, LaTeX thesis sources, and documentation to reproduce results and run the robot.

### Pictures
- Primary figures and dataset images live in `Images` and `JetsonNano/Images`.

### Folder structure (with GitHub links)

#### Core Directories

| Directory | Purpose |
|-----------|---------|
| [Assembly](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Assembly) | LaTeX sources for assembly/hardware documentation with build artifacts (PDFs, auxiliary files) and organized chapters in German and English |
| [Code](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Code) | Main software projects: JetBot robot code, MicroPython examples, and SolidWorks CAD exports |
| [JetsonNano](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano) | Jetson Nano-specific code, TensorFlow/TFLite experiments, system configurations, and utilities |
| [Manual](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Manual) | Step-by-step setup and operation guides (DeveloperManual, UserManual) |
| [MLbib](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/MLbib) | Bibliography and literature references organized by topic (ML, Localization, RaspberryPi, etc.) |
| [Thesis](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Thesis) | Main thesis LaTeX document, contents, figures, and TikZ graphics |
| [Presentations](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Presentations) | Presentation slides covering topics like localization, hardware, ROS2, SLAM, and thesis colloquium |
| [Poster](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Poster) | Conference/academic poster sources and exported images |
| [ProjectManagement](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/ProjectManagement) | Project planning, schedules, and management documents |

#### Code Subdirectories

| Path | Contents |
|------|----------|
| [Code/JetBot](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Code/JetBot) | ROS2 robot software with CMakeLists, package files, launch configurations, models, and experiment scripts |
| [Code/DomainSystem](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Code/DomainSystem) | Domain-specific system implementations and test utilities |
| [Code/MicroPython](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Code/MicroPython) | MicroPython examples for microcontroller integration |
| [Code/Solidworks](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Code/Solidworks) | CAD exports from SolidWorks (3D models, G-code for 3D printing) |

#### JetsonNano Subdirectories

| Path | Contents |
|------|----------|
| [JetsonNano/Code](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/Code) | Experiment implementations: Arduino integration, TensorFlow Lite, Nano 33 BLE Sense support |
| [JetsonNano/System](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/System) | System configuration and setup scripts |
| [JetsonNano/Contents](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/Contents) | Data and content files |
| [JetsonNano/Images](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/Images) | Experimental results and dataset images |
| [JetsonNano/tikz](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/tikz) | TikZ graphics for visualizations |

#### Presentation Subdirectories

| Path | Contents |
|------|----------|
| Presentations/Localization | Localization algorithm and approach presentations |
| Presentations/GroundLocalizationAlgorithm | Specific ground-based localization algorithm details |
| Presentations/SLAM | SLAM (Simultaneous Localization and Mapping) techniques |
| Presentations/ROS2 | ROS2 framework and integration |
| Presentations/NVIDIAJetsonNano | Jetson Nano hardware and capabilities |
| Presentations/RaspberryPiCameraModule2 | Camera module specifications and usage |
| Presentations/LiteratureReview | Academic literature and related work review |
| Presentations/ThesisColloquiumPresentation | Thesis colloquium presentation materials |

### Short descriptions of key folders

- **Assembly/**: Contains LaTeX documentation for assembly procedures. Organized with Chapters (German and English), General macros/commands, Images for hardware documentation, and compiled PDF outputs.
- **Code/JetBot/**: Primary robot control software using ROS2/ROS. Contains CMakeLists for build configuration, package.xml for dependencies, launch files for robot startup, trained models, configuration files, and Python/C++ scripts for experiments.
- **Code/DomainSystem/**: Domain-specific implementations and testing utilities.
- **Code/MicroPython/**: Microcontroller code examples written in MicroPython.
- **Code/Solidworks/**: CAD model exports and G-code files for 3D-printed bumper components.
- **JetsonNano/Code/**: Experiment implementations including Arduino integration, TensorFlow Lite runners, and Nano 33 BLE Sense support.
- **JetsonNano/System/**: System configuration files and initialization scripts for the Jetson Nano platform.
- **JetsonNano/Images/**: Experimental result images and dataset frames collected during localization trials.
- **JetsonNano/tikz/**: TikZ graphics and visualizations used in documentation.
- **Manual/**: User and developer manuals with setup instructions, calibration procedures, and operational guides.
- **MLbib/**: Organized bibliography entries by topic (Machine Learning, Localization, Mobile Robots, SLAM, RaspberryPi, Python, etc.).
- **Thesis/**: Main thesis document in LaTeX format (LocalizationofaMobileRobot.tex), organized with Contents, General setup, thesis Images, and TikZ graphics.
- **Presentations/**: Presentation slides organized by topic (Localization algorithms, hardware, ROS2, SLAM, literature review, etc.).
- **Poster/**: Academic poster document and exported images for conferences.
- **ProjectManagement/**: Project planning, timeline, and management documents.

### Important files (with GitHub links and brief descriptions)

#### Documentation & Project Setup
- [README.md](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/README.md): Main project overview and repository guide
- [author.xlsx](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/author.xlsx): Author and contributor information

#### Thesis & Academic Documents
- [Thesis/LocalizationofaMobileRobot.tex](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Thesis/LocalizationofaMobileRobot.tex): Main thesis document (primary academic work)
- [Thesis/LocalizationofaMobileRobot.pdf](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Thesis/LocalizationofaMobileRobot.pdf): Compiled thesis document
- [Assembly/MobileRobotAssembly.tex](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Assembly/MobileRobotAssembly.tex): Hardware assembly documentation
- [Assembly/MobileRobotAssembly.pdf](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Assembly/MobileRobotAssembly.pdf): Compiled assembly manual
- [Poster/tikzposter.tex](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Poster/tikzposter.tex): Academic poster source

#### Robot Software & Configuration
- [Code/JetBot/CMakeLists.txt](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/JetBot/CMakeLists.txt): Build configuration for JetBot codebase
- [Code/JetBot/package.xml](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/JetBot/package.xml): ROS2 package dependencies and metadata
- [Code/JetBot/README.md](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/JetBot/README.md): JetBot-specific setup and usage instructions
- [Code/JetBot/requirements.txt](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/JetBot/requirements.txt): Python dependencies for robot software
- [Code/JetBot/run_jetbot_all.sh](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/JetBot/run_jetbot_all.sh): Main startup script for robot initialization

#### Experiment & Implementation Code
- [Code/DomainSystem/Test.py](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/DomainSystem/Test.py): Domain system testing and validation
- [JetsonNano/Code/JetsonNano/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/JetsonNano/Code/JetsonNano): Localization and collision avoidance experiment implementations

#### Bibliography & References
- [MLbib/Jetson.bib](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/MLbib/Jetson.bib): NVIDIA Jetson references
- [MLbib/Literature.bib](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/MLbib/Literature.bib): Main literature references
- [MLbib/MyLiterature.bib](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/MLbib/MyLiterature.bib): Custom literature entries
- [MLbib/Localization/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/MLbib/Localization): Localization-specific bibliography
- [MLbib/ML/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/MLbib/ML): Machine learning references
- [MLbib/MobileRobot/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/MLbib/MobileRobot): Mobile robotics references

#### Hardware & CAD
- [Code/Solidworks/RobotBumper.SLDPRT](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/Solidworks/RobotBumper.SLDPRT): Bumper component SolidWorks design file
- [Code/Solidworks/RobotBumper_0.2mm_PLA_MK3_45m.gcode](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/blob/main/Code/Solidworks/RobotBumper_0.2mm_PLA_MK3_45m.gcode): 3D print G-code for bumper

#### Manuals
- [Manual/DeveloperManual/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Manual/DeveloperManual): Developer setup and programming guides
- [Manual/UserManual/](https://github.com/Wings-hub/250901-Jetson-Nano-Ground/tree/main/Manual/UserManual): End-user operation and safety guides

---

## Hardware Components

### Primary Platform
- **SparkFun JetBot**: Mobile robotic platform with differential drive
- **NVIDIA Jetson Nano**: Edge computing device for onboard processing
- **Waveshare 16578 Wireless-AC8265**: WiFi module (AC standard, replaces default USB WiFi adapter)

### Compatible Sensors & Modules
- Arduino-compatible microcontrollers for sensor integration
- Nano 33 BLE Sense (optional sensor suite)
- Raspberry Pi Camera Module 2 (for vision-based localization)
- TensorFlow Lite support for onboard inference

---

## Bill of Materials (SparkFun JetBot Components)

### Compute Module

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| NVIDIA Jetson Nano | DEV-16271 | 1 | 4GB RAM, 128-core GPU, ARM Cortex-A57 |
| Jetson Nano Developer Kit Carrier | Included | 1 | GPIO, USB, Ethernet, microSD slot |

### Power Management

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Li-Ion Battery Pack | PRT-16059 | 1 | 14.8V, 2600mAh, 38.48Wh (typical JetBot pack) |
| Battery Charging Circuit | PRT-18719 | 1 | 2A charging, protection circuitry |
| Power Distribution Board (PDB) | ROB-13046 | 1 | Voltage regulation, current protection |
| 5V/4A USB-C Power Supply | COM-16271 | 1 | Alternative desktop power option |
| Micro USB to DC Barrel Jack | PRT-10285 | 1 | Secondary charging connector |

### Motor & Drivetrain

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| DC Motor (TT Motor) | ROB-16033 | 2 | 3-6V, ~150rpm, gearbox included |
| Wheel (TT Motor compatible) | ROB-16035 | 2 | 65mm diameter, rubber treads |
| Castor Wheel (Ball Caster) | ROB-11571 | 1 | 5/16" ball, swivel mount |
| Motor Driver Module (DRV8835) | ROB-14450 | 1 | Dual channel, PWM speed control |
| Motor Encoder (Optical) | ROB-12629 | 2 | Rotational speed feedback |
| Mounting Brackets (Motor) | PRT-13183 | 2 | Aluminum, secure motor installation |

### Mechanical Components

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Robot Chassis (Plastic) | ROB-16426 | 1 | Multi-tier 3-layer platform |
| Spacers & Standoffs | PRT-10917 | 10-20 | Plastic, M3 threaded |
| Servo Mount (Standard) | PRT-10336 | 2 | Universal servo bracket |
| Servo Motor (SG90) | ROB-09065 | 1-2 | Pan/tilt control, optional |
| Mounting Tape (Double-sided) | COM-15130 | 1 | For sensor mounting flexibility |
| Hardware Kit (Screws, Nuts, Bolts) | - | 1 | M2, M3, M4 assortment |

### Camera & Vision

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Raspberry Pi Camera Module v2 | BOB-14028 | 1 | 8MP, 1080p video, IMX219 sensor |
| Camera Cable (15cm ribbon) | BOB-14000 | 1 | CSI interface, 15cm length |
| Camera Mount | PRT-14319 | 1 | Adjustable angle bracket |
| USB Webcam (alternative) | COM-18150 | 1 | 1080p, USB interface option |

### Communication & Connectivity

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Waveshare WiFi Card | 16578 | 1 | Intel 8265, 802.11ac, M.2 slot |
| Micro SD Card | - | 1 | 64GB minimum, Class 10, A1 rating |
| USB Hub (4-port) | COM-12642 | 1 | For multiple USB peripherals |
| USB Serial Cable | COM-15172 | 1 | CH340 chipset for Arduino communication |
| Jumper Wires (M/M) | PRT-11375 | 1 pack | Various lengths, 0.1" pitch |
| Jumper Wires (F/M) | PRT-12794 | 1 pack | For sensor connections |
| GPIO Header Pins | PRT-00116 | 1 pack | Breakout headers for expansion |

### Microcontroller Integration

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Arduino Nano (or compatible) | DEV-13813 | 1 | ATmega328P, 16MHz, PWM outputs |
| Arduino Uno (alternative) | DEV-11224 | 1 | Larger form factor, more I/O |
| FTDI USB Serial Adapter | BOB-00718 | 1 | Backup serial communication |

### Power & Safety

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Fuse Holder (Inline) | PRT-11850 | 1 | 3-5A protection |
| Power Switch (SPDT) | COM-11121 | 1 | Manual power control |
| LED Status Indicator | COM-12062 | 2 | Power and status monitoring |
| Capacitor (Electrolytic, 100µF) | COM-00523 | 3 | Voltage smoothing |
| Resistor Pack (assorted) | COM-10969 | 1 | Current limiting, various values |

### Miscellaneous

| Component | Part Number | Quantity | Specifications |
|-----------|-------------|----------|-----------------|
| Heat Sink (Jetson Nano) | PRT-14614 | 1 | Passive thermal management |
| Thermal Paste | COM-09163 | 1 | Heat sink application |
| Cable Ties/Velcro Straps | PRT-15206 | 1 pack | Cable management |
| Label Maker Tape | COM-12689 | Optional | Organization and documentation |

### Assembly Tools (Not Included, Recommended)

- Soldering iron (30-40W)
- Solder (lead-free, rosin core)
- Hot glue gun
- Multimeter (digital)
- Wire strippers/crimpers
- Mini screwdriver set
- Tweezers (anti-static)

---

## Alternative Components & Upgrades

### WiFi Options
- **Waveshare 16578** (Implemented): 802.11ac, M.2 M-Key slot
- **USB WiFi Dongle** (Standard JetBot): 802.11b/g/n, easier replacement
- **Ethernet USB Adapter**: Wired gigabit connection for stable testing

### Motor Upgrades
- **Brushless Motors**: Higher efficiency, longer lifespan
- **Stepper Motors**: Precise step control for positioning
- **Servo Motors**: Enhanced pan/tilt capabilities

### Camera Alternatives
- **Raspberry Pi HQ Camera**: 12MP, CS-mount lens
- **Intel Realsense D435**: Depth camera for 3D localization
- **OAK-D**: OpenCV AI Kit with depth and spatial AI

### Battery Options
- **LiPo Battery**: Higher energy density, shorter charge time
- **NiMH Battery**: Safer, longer cycle life
- **USB Power Bank**: Emergency backup power

---

## Wiring & Connector Reference

### GPIO Pin Mapping (40-pin header)

```
Motor Control (GPIO)
├── Motor Left PWM:     GPIO 18 (PWM0)
├── Motor Left Dir:     GPIO 27
├── Motor Right PWM:    GPIO 13 (PWM1)
└── Motor Right Dir:    GPIO 17

I2C Communication
├── SDA:                GPIO 2  (I2C1)
└── SCL:                GPIO 3  (I2C1)

UART Serial
├── TX:                 GPIO 14 (UART0)
└── RX:                 GPIO 15 (UART0)

SPI Communication
├── MOSI:               GPIO 10 (SPI0)
├── MISO:               GPIO 9  (SPI0)
├── CLK:                GPIO 11 (SPI0)
└── CE:                 GPIO 8  (SPI0)

Sensor Inputs
├── Ultrasonic Echo:    GPIO 24, 25 (configurable)
├── Ultrasonic Trigger: GPIO 23, 26 (configurable)
└── Line Sensor:        GPIO 16, 20 (configurable)
```

### Connector Types Used

- **Motor Connectors**: JST PH 2.0 mm (2-pin)
- **Battery Connector**: XT60 or JST LiPo (2-pin)
- **Camera Interface**: 15-pin CSI ribbon (Jetson Nano)
- **USB Ports**: USB 3.0 Type-A (Jetson Nano)
- **GPIO Header**: 40-pin 0.1" pitch

---

## Implementation Details

### Collision Avoidance System

The collision avoidance module provides autonomous obstacle detection and navigation:

#### Object Detection Pipeline

```
Camera Input → Image Preprocessing → DNN Inference → Obstacle Detection → 
Motor Control Signal → Motion Planning
```

#### Components

- **Real-time Detection**: Using TensorFlow Lite optimized models (MobileNetSSD, YOLO-Nano)
- **Distance Estimation**: Monocular depth prediction from single camera frame
- **Behavioral Response**: Velocity scaling, steering adjustment, complete stop logic
- **Safety Boundaries**: Configurable safety zones in pixels/meters

#### Configuration

```yaml
# JetsonNano/System/collision_avoidance.yaml
detection:
  model: "mobilenet_ssd"
  confidence_threshold: 0.6
  fps: 30

depth_estimation:
  method: "monocular"
  range_meters: [0.2, 3.0]

avoidance:
  min_safe_distance_cm: 30
  emergency_stop_distance_cm: 15
  turning_speed: 0.3
```

#### Testing & Validation

```bash
# Test collision avoidance in controlled environment
python JetsonNano/Code/JetsonNano/test_collision_avoidance.py

# Generate avoidance behavior logs
python JetsonNano/Code/JetsonNano/collision_test_suite.py --output logs/avoidance/
```

---

### Monocular Localization System

Single camera-based localization using visual features and ego-motion estimation:

#### Monocular Localization Pipeline

```
Camera Frame (t) → Feature Extraction → Feature Matching → 
Pose Estimation → Localization Map Update → Position Estimate (t+1)
```

#### Techniques Implemented

| Technique | Implementation | File Location |
|-----------|-----------------|----------------|
| **Feature Detection** | ORB, SIFT, SURF features | `JetsonNano/Code/JetsonNano/localization/features.py` |
| **Visual Odometry** | Ego-motion tracking frame-to-frame | `JetsonNano/Code/JetsonNano/localization/visual_odometry.py` |
| **Keyframe Management** | Bundle adjustment and loop closure | `JetsonNano/Code/JetsonNano/localization/keyframe_manager.py` |
| **Map Representation** | Sparse 3D landmarks, dense depth maps | `JetsonNano/Code/JetsonNano/localization/map_representation.py` |

#### Localization Accuracy Metrics

- **Drift Error**: Position error accumulation over trajectory (m/s)
- **Absolute Trajectory Error (ATE)**: Deviation from ground truth
- **Relative Pose Error (RPE)**: Frame-to-frame pose accuracy

#### Calibration Requirements

```bash
# Camera intrinsics calibration (performed once)
python JetsonNano/Code/calibrate_camera.py --images data/calibration/ --output calib/camera.yaml

# Stereo rectification (if using binocular setup)
python JetsonNano/Code/calibrate_stereo.py --left-images data/left/ --right-images data/right/
```

#### Running Monocular Localization

```bash
# Single image test
python JetsonNano/Code/JetsonNano/monocular_localization.py --image test.jpg

# Video stream processing
python JetsonNano/Code/JetsonNano/monocular_localization.py --video stream --output trajectory.txt

# Real-time on robot camera
python JetsonNano/Code/JetsonNano/localization/run_online.py --mode monocular
```

---

### Dataset Development

Comprehensive dataset for training and evaluating collision avoidance and localization algorithms:

#### Dataset Structure

```
JetsonNano/Images/
├── Dataset/
│   ├── CollisionAvoidance/
│   │   ├── raw/                 # Raw video streams
│   │   ├── annotated/           # Labeled obstacle locations
│   │   │   ├── images/          # Frames with bounding boxes
│   │   │   └── annotations.json # COCO format annotations
│   │   ├── splits/
│   │   │   ├── train.txt        # 70% of dataset
│   │   │   ├── val.txt          # 15% of dataset
│   │   │   └── test.txt         # 15% of dataset
│   │   └── metadata.yaml        # Dataset statistics
│   │
│   └── Localization/
│       ├── Trajectories/        # Ground truth trajectories
│       │   ├── indoor/
│       │   └── outdoor/
│       ├── Keyframes/           # Key camera frames with poses
│       ├── DepthMaps/           # Ground truth depth (when available)
│       ├── Camera/              # Camera initialization sequences
│       └── Calibration/         # Camera calibration data
```

#### Data Collection Protocol

**Collision Avoidance Dataset:**

```bash
# Collect raw data
python JetsonNano/Code/JetsonNano/dataset/collect_collision_data.py \
  --duration 3600 \
  --output data/raw/ \
  --obstacles mixed

# Annotate obstacles
python JetsonNano/Code/JetsonNano/dataset/annotate_obstacles.py \
  --input data/raw/ \
  --output data/annotated/ \
  --format coco

# Generate dataset splits
python JetsonNano/Code/JetsonNano/dataset/create_splits.py \
  --input data/annotated/ \
  --train-ratio 0.7 \
  --val-ratio 0.15
```

**Localization Dataset:**

```bash
# Record camera trajectory
python JetsonNano/Code/JetsonNano/dataset/record_trajectory.py \
  --duration 600 \
  --fps 30 \
  --output data/trajectories/

# Extract keyframes
python JetsonNano/Code/JetsonNano/dataset/extract_keyframes.py \
  --input data/trajectories/ \
  --output data/keyframes/ \
  --interval 5

# Generate ground truth poses (manual annotation or motion capture)
python JetsonNano/Code/JetsonNano/dataset/generate_ground_truth.py \
  --keyframes data/keyframes/ \
  --output data/poses/
```

#### Dataset Statistics

```yaml
CollisionAvoidance:
  total_frames: 108000
  image_resolution: 640x480
  fps: 30
  duration_hours: 1.0
  environments: [indoor_static, indoor_dynamic, outdoor]
  obstacle_classes: [person, wall, furniture, vehicle, animal]
  training_samples: 75600
  validation_samples: 16200
  test_samples: 16200

Localization:
  trajectory_count: 50
  total_distance_m: 2500
  indoor_sequences: 30
  outdoor_sequences: 20
  camera_fps: 30
  total_keyframes: 5000
  depth_coverage: 40%
```

#### Data Formats

- **Images**: PNG/JPEG (lossless preferred)
- **Annotations**: COCO JSON, Pascal VOC XML
- **Trajectories**: TUM RGB-D format
- **Metadata**: YAML configuration files
- **Sensor Data**: ROS Bagfiles, HDF5

#### Quality Assurance

```bash
# Validate dataset integrity
python JetsonNano/Code/JetsonNano/dataset/validate.py --dataset data/

# Check for duplicates and corrupted files
python JetsonNano/Code/JetsonNano/dataset/check_quality.py --input data/

# Generate dataset report
python JetsonNano/Code/JetsonNano/dataset/generate_report.py --output report.html
```

#### Dataset Usage Examples

```python
from dataset_loader import CollisionAvoidanceDataset, LocalizationDataset

# Load collision avoidance dataset
collision_dataset = CollisionAvoidanceDataset(
    root_dir='data/CollisionAvoidance/',
    split='train',
    transform=transforms.Compose([...])
)

# Load localization dataset
localization_dataset = LocalizationDataset(
    root_dir='data/Localization/',
    sequence_length=5
)

# Iterate through samples
for images, annotations in collision_dataset:
    # Process batch
    model_output = model(images)
```

#### Sharing & Reproducibility

```bash
# Create dataset archive
tar -czf jetbot_collision_avoidance_dataset_v1.tar.gz data/CollisionAvoidance/

# Create dataset metadata for publication
python JetsonNano/Code/JetsonNano/dataset/create_metadata.py \
  --dataset data/ \
  --output dataset_metadata.bibtex
```

---

```
├── Assembly/                 # Hardware assembly documentation (LaTeX)
│   ├── MobileRobotAssembly.tex
│   └── Chapters/            # Assembly chapters (multilingual)
│
├── Code/                    # Source code for various platforms
│   ├── Arduino/             # Arduino sketches
│   ├── MicroPython/         # MicroPython implementations
│   └── Solidworks/          # 3D models and GCODE files
│
├── JetsonNano/              # Jetson Nano specific resources
│   ├── Code/                # Jetson Nano applications
│   │   ├── JetsonNano/      # Main Jetson code
│   │   ├── TensorFlowLite/  # ML inference examples
│   │   ├── Arduino/         # Serial communication code
│   │   └── Nano33BLESense/  # Sensor integration
│   ├── Images/              # Documentation and reference images
│   ├── System/              # System configuration files
│   └── tikz/                # TikZ diagrams for documentation
│
├── Manual/                  # Project documentation
│   ├── DeveloperManual/     # Development guide
│   └── UserManual/          # User guide
│
├── MLbib/                   # Bibliography and references
│   ├── Jetson.bib
│   ├── Literature.bib
│   └── Topic-specific bib files
│
├── Thesis/                  # Main thesis document
│   └── LocalizationofaMobileRobot.tex
│
├── Presentations/           # Presentation materials
│   ├── Localization/
│   ├── SLAM/
│   ├── ROS2/
│   └── Other topics
│
└── ProjectManagement/       # Project planning documents
```

## WiFi Card Setup (Waveshare 16578)

The Waveshare 16578 Wireless-AC8265 WiFi card provides:
- **Standard**: 802.11ac (WiFi 5)
- **Speed**: Faster connectivity for data transfer and remote control
- **Installation**: Replaces the default USB WiFi adapter on the Jetson Nano

### Setup Instructions

1. **Physical Installation**: Insert the WiFi card into the appropriate M.2 slot
2. **Driver Installation**: 
   ```bash
   sudo apt update
   sudo apt install firmware-iwlwifi
   ```
3. **Verify Connection**:
   ```bash
   iwconfig
   nmtui  # For network management
   ```

## Key Features & Algorithms

This project implements and evaluates:
- **Localization Algorithms**: Position estimation and tracking
- **SLAM Integration**: Simultaneous Localization and Mapping
- **Computer Vision**: Camera-based environment perception
- **Edge AI**: TensorFlow Lite inference on Jetson Nano
- **ROS 2 Integration**: Robot Operating System framework

## Getting Started

### Prerequisites
- NVIDIA Jetson Nano with JetPack installed
- SparkFun JetBot kit fully assembled
- Waveshare 16578 WiFi module installed
- Arduino IDE (for microcontroller programming)
- Python 3.8+

### Quick Start

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Wings-hub/250901-Jetson-Nano-Ground.git
   cd 250901-Jetson-Nano-Ground
   ```

2. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run Main Application**:
   ```bash
   cd JetsonNano/Code
   python main.py
   ```

## Assembly Guide

### Hardware Components Assembly

Complete assembly instructions are available in the LaTeX documentation:
- **Main Document**: `Assembly/MobileRobotAssembly.tex`
- **multilingual Documentation**: Assembly chapters in `Assembly/Chapters/(de|en)/`

#### Assembly Steps

1. **Mechanical Assembly**
   - Assemble SparkFun JetBot chassis and motor mounts
   - Install wheels and motor encoders
   - Mount Jetson Nano compute module securely
   - Install camera module with appropriate brackets

2. **Electrical Integration**
   - Connect motor drivers to Jetson Nano GPIO pins
   - Wire Power Distribution Board (PDB)
   - Install power supply (5V/4A recommended)
   - Connect sensor interfaces (I2C, UART, SPI)
   - Install Waveshare 16578 WiFi card into M.2 slot

3. **Sensor Calibration**
   - Camera calibration matrices in `JetsonNano/Images/camcalib/`
   - IMU sensor calibration procedures
   - Motor encoder verification

4. **3D Printed Components**
   - Solidworks models: `Code/Solidworks/`
   - Pre-sliced GCODE files available
   - Alternative materials and print settings documented

### Schematic & Wiring Diagrams

- Motor and servo connections
- Power distribution layout
- Sensor interface pinouts
- Serial communication wiring (Arduino ↔ Jetson)

---

## User Manual

### Quick Operation Guide

Complete user documentation available in `Manual/UserManual/`

#### Starting the Robot

```bash
# SSH into Jetson Nano
ssh jetson@<robot-ip>

# Navigate to application directory
cd JetsonNano/Code/JetsonNano

# Run the main application
python main.py
```

#### Remote Control

1. **WiFi Connection**: Connect to robot's WiFi network via Waveshare 16578 adapter
2. **Web Interface**: Access control dashboard at `http://<robot-ip>:8080`
3. **Keyboard Control**: Use arrow keys for movement
4. **Sensor Monitoring**: View real-time telemetry

#### Camera & Vision

```bash
# Test camera functionality
python JetsonNano/Code/test_camera.py

# Run object detection
python JetsonNano/Code/TensorFlowLite/object_detection.py

# Camera calibration
python JetsonNano/Code/calibrate_camera.py
```

#### Data Collection

- Sensor logs stored in `~/data/logs/`
- Camera frames saved to `~/data/images/`
- Localization data recorded in `~/data/localization/`
- Export formats: CSV, HDF5, ROS bagfiles

#### Safety Considerations

- Always ensure adequate space before autonomous operation
- Monitor battery level constantly
- Verify WiFi connection stability
- Implement collision detection boundaries
- Emergency stop procedures documented

#### Troubleshooting

**Robot Won't Connect**
```bash
# Check wireless connection
iwconfig
# Restart network manager
sudo systemctl restart network-manager
```

**Slow Performance**
```bash
# Check Jetson Nano temperature
tegrastats
# Monitor CPU/GPU usage
jtop
```

**Sensor Issues**
```bash
# Check I2C devices
i2cdetect -y -r 1
# Test serial connection
minicom -D /dev/ttyTHS1 -b 115200
```

---

## Developer Manual

### Development Environment Setup

Complete development documentation available in `Manual/DeveloperManual/`

#### Prerequisites Installation

```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Install development tools
sudo apt install build-essential python3-dev python3-pip git cmake

# Install JetPack components (if needed)
sudo apt install cuda-toolkit-11-4 libcudnn8 tensorrt
```

#### Python Development Environment

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install project dependencies
pip install -r requirements.txt
```

### Project Structure for Developers

#### Core Modules

| Module | Location | Purpose |
|--------|----------|---------|
| **Localization** | `JetsonNano/Code/JetsonNano/localization/` | Main localization algorithms |
| **Vision** | `JetsonNano/Code/JetsonNano/vision/` | Camera and image processing |
| **Motor Control** | `JetsonNano/Code/JetsonNano/motors/` | Motor driver interface |
| **Sensors** | `JetsonNano/Code/JetsonNano/sensors/` | Sensor data acquisition |
| **Utils** | `JetsonNano/Code/JetsonNano/utils/` | Helper functions |

#### Communication Layers

```
┌─────────────────────────────────┐
│   High-Level Localization API   │
└──────────────┬──────────────────┘
               │
┌──────────────▼──────────────────┐
│  Algorithm Implementation Layer  │
├──────────────┬──────────────────┤
│  Vision      │    Sensor Data   │
│  Processing  │    Processing    │
└──────────────┼──────────────────┘
               │
┌──────────────▼──────────────────┐
│    Hardware Abstraction Layer    │
├──────────────┬──────────────────┤
│   Serial     │      GPIO        │
│   (Arduino)  │   (Motors)       │
└──────────────┴──────────────────┘
```

### Key Development Files

#### Main Application Entry Points

- `JetsonNano/Code/JetsonNano/main.py` - Primary application
- `JetsonNano/Code/JetsonNano/run_localization.py` - Standalone localization module
- `JetsonNano/Code/test_*.py` - Unit tests for each component

#### Configuration

- `JetsonNano/System/config.json` - Runtime configuration
- `JetsonNano/System/JetsonNano/params.yaml` - ROS 2 parameters
- `Code/Arduino/config.h` - Microcontroller settings

### Development Workflow

#### Building & Testing

```bash
# Run unit tests
pytest tests/ -v

# Run integration tests
python JetsonNano/Code/integration_test.py

# Profile performance
python -m cProfile -s cumtime JetsonNano/Code/main.py

# Memory profiling
python -m memory_profiler JetsonNano/Code/main.py
```

#### Code Style & Quality

```bash
# Code formatting
black JetsonNano/Code/

# Linting
pylint JetsonNano/Code/

# Type checking
mypy JetsonNano/Code/
```

#### Arduino Development

```bash
# Compile Arduino code
cd Code/Arduino
arduino-cli compile --fqbn arduino:avr:nano Blink/

# Upload to device
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano Blink/
```

### Algorithm Implementation Guide

#### Adding a New Localization Algorithm

1. Create algorithm module in `JetsonNano/Code/JetsonNano/localization/algorithms/`
2. Implement standard interface (init, predict, update methods)
3. Add configuration parameters to `JetsonNano/System/config.json`
4. Create test cases in `tests/test_algorithm.py`
5. Update documentation with algorithm details

#### Performance Optimization

- Use NumPy for vectorized operations
- Profile bottlenecks with `cProfile`
- Consider CUDA acceleration for heavy computations
- Implement edge caching for frequently used data

### Debugging & Logging

```bash
# Enable detailed logging
export LOG_LEVEL=DEBUG
python main.py

# View Jetson system logs
journalctl -u your-service -f

# Real-time monitoring
jtop

# Thermal monitoring
while true; do tegrastats; sleep 1; done
```

### Continuous Integration

- Unit tests in `tests/` directory
- Integration tests for hardware components
- Docker builds for reproducible environments
- Automated testing on each commit

### Performance Benchmarks

Monitor and document:
- Algorithm execution time (ms)
- Memory usage (MB)
- CPU/GPU utilization (%)
- Temperature during operation
- Power consumption (W)

Benchmarks stored in `JetsonNano/Images/Test/benchmarks/`

---

## Key Directories

- **`JetsonNano/Code/`**: Core localization algorithms and applications
- **`JetsonNano/Images/`**: Calibration data, test images, and reference materials
- **`Thesis/`**: Complete thesis document with research findings
- **`Presentations/`**: Conference/seminar presentation materials
- **`Code/Arduino/`**: Microcontroller integration code
- **`Manual/`**: Complete documentation for developers and users

## Important Files

- `Thesis/LocalizationofaMobileRobot.tex`: Main thesis document
- `JetsonNano/Code/JetsonNano/`: Primary Jetson Nano application code
- `Code/Arduino/`: Arduino sketches for sensor/actuator control
- `JetsonNano/Contents/`: Algorithm documentation and theory

## Dependencies

### System Libraries
- NVIDIA JetPack (CUDA, cuDNN, TensorRT)
- OpenCV 4.x
- ROS 2 (optional, for robotic framework)

### Python Packages
- TensorFlow Lite
- NumPy
- SciPy
- PySerial (for Arduino communication)
- OpenCV (cv2)

## Communication Protocols

- **UART/Serial**: Arduino ↔ Jetson Nano
- **WiFi**: Remote control and data transmission (Waveshare 16578)
- **USB**: Development and programming

## Documentation

- **LaTeX Documents**: Assembly and thesis in `Assembly/` and `Thesis/`
- **Developer Manual**: Complete development guidelines in `Manual/DeveloperManual/`
- **User Manual**: Usage instructions in `Manual/UserManual/`
- **Code Examples**: Sample implementations in `JetsonNano/Code/`

## Research Topics Covered

- **Localization Algorithms**: Various approaches and comparisons
- **Camera Calibration**: OpenCV-based calibration methods
- **SLAM**: Simultaneous localization and mapping techniques
- **ROS 2**: Robotic middleware integration
- **TensorFlow Lite**: Efficient ML inference on edge devices
- **Internet of Things (IoT)**: Connected robotic systems

## Testing & Validation

- Test images and data in `JetsonNano/Images/Test/`
- Calibration datasets in `JetsonNano/Images/camcalib/`
- Example camera outputs in `JetsonNano/Images/Camera/`
- TensorFlow evaluation datasets in `JetsonNano/Images/TensorFlow/`

## Building the Project

### LaTeX Compilation
```bash
cd Thesis/
pdflatex LocalizationofaMobileRobot.tex
bibtex LocalizationofaMobileRobot
pdflatex LocalizationofaMobileRobot.tex
```

### Running Applications
```bash
cd JetsonNano/Code/JetsonNano
python your_script.py
```

## Troubleshooting

### WiFi Connection Issues
- Verify driver installation: `iwconfig`
- Check WiFi scan: `sudo nmtui`
- View system logs: `dmesg | grep -i wifi`

### Jetson Nano Issues
- Monitor temperature: `tegrastats`
- Check power supply (5A recommended minimum)
- Verify JetPack version: `dpkg -l | grep jetson`

### Python Dependencies
- Update pip: `pip install --upgrade pip`
- Install missing packages: `pip install -r requirements.txt`

## License

This project is part of academic research. Please see project documentation for licensing information.

## Contact & Support

For questions or issues regarding this project:
- **Author**: Srikanth Nanda
- **Project Repository**: [GitHub Repository](https://github.com/Wings-hub/250901-Jetson-Nano-Ground)

## References

Complete bibliographic references are available in:
- `MLbib/Literature.bib` - General literature
- `MLbib/Jetson.bib` - Jetson-specific references
- `MLbib/Algorithm/` - Algorithm-specific papers
- `Thesis/LocalizationofaMobileRobot.bib` - Thesis bibliography

## Acknowledgments

- NVIDIA Jetson community and documentation
- SparkFun JetBot project
- Waveshare and hardware manufacturers
- University/Institution support

---

**Last Updated:** April 2026  
**Status:** Active Development