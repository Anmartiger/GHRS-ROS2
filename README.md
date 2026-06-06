<div align="center">



# 🌿 GHRS · Green House Rover System

### *Autonomous Green House Monitoring, Disease Detection & Pesticide Spraying Rover*

> **Developer:** Anmar Arafat Al-Momani   
> **Platform:** Raspberry Pi 5 · Ubuntu · ROS2 Jazzy

<img src="https://i.imgur.com/SSGXqgy.png" alt="GHRS Rover" width="600"/>

</div>

---

## Overview

GHRS is an autonomous ground rover designed for greenhouse environments. It navigates along configurable GPS waypoints, monitors plant health using computer vision, and autonomously sprays pesticide on diseased plants using a pan-tilt hose turret. All perception, navigation, and actuation runs as a ROS2 Jazzy node graph on a Raspberry Pi 5.

| Capability | Method |
|---|---|
| 🗺️ Waypoint Patrol | GPS (NEO-6M) + IMU heading |
| 🌿 Plant Disease Detection | CNN / colour heuristic fallback |
| 💧 Pesticide Spraying | Pan-tilt turret + pump |
| 🚧 Obstacle Avoidance | Camera edge density |
| 📊 Mission Reports | HTML + JSON auto-generation |
| 🎥 Live Stream | MJPEG on port 5001 |

---

## System Architecture

<img src="https://i.imgur.com/topf1nW.png" alt="GHRS Rover" width="600"/>

# GHRS ROS2 Node Graph

The **Greenhouse Rover System (GHRS)** is an autonomous agricultural rover built on Raspberry Pi 5. Its ROS2 architecture flows through four layers:

- **Sensors & Input** — Camera, IMU, and GPS nodes feed raw data into the system.
- **Processing** — Plant disease detection, PID-based camera tracking, and GPS waypoint navigation.
- **Actuators** — Servo controller for turret aim and motor controller for wheel drive.
- **Spray System** — Coordinates turret targeting and pump firing for precision pesticide delivery.

All subsystems report to a central `report_node` that outputs a full mission log as HTML/JSON.

---

## Package Structure

```
ghrs_ros2/
├── ghrs_msgs/               # Custom messages & services
│   ├── msg/
│   │   ├── RoverStatus.msg
│   │   ├── FireDetection.msg
│   │   ├── ServoCommand.msg
│   │   └── PlantHealth.msg
│   └── srv/
│       └── DriveCommand.srv
│
├── ghrs_hardware/           # Hardware driver nodes
│   └── ghrs_hardware/
│       ├── motor_controller_node.py   # BTS7960 differential drive
│       ├── servo_controller_node.py   # PCA9685 all-servo control
│       ├── imu_node.py                # BNO08x IMU publisher
│       ├── gps_node.py                # NEO-6M NMEA GPS
│       ├── camera_node.py             # Webcam + IP cam + MJPEG server
│       └── pump_led_node.py           # GPIO pump and LED
│
├── ghrs_perception/         # Computer vision nodes
│   └── ghrs_perception/
│       ├── turret_tracking_node.py    # PID pan-tilt tracker
│       ├── obstacle_detection_node.py # Camera edge-density obstacles
│       └── plant_disease_node.py      # Plant disease classifier
│
├── ghrs_navigation/         # Autonomous navigation
│   └── ghrs_navigation/
│       └── navigation_node.py         # GPS patrol + obstacle avoidance
│
├── ghrs_fire/               # Pesticide spraying coordinator
│   └── ghrs_fire/
│       └── fire_suppression_node.py   # Spray orchestration node
│
├── ghrs_reporting/          # Mission report generation
│   └── ghrs_reporting/
│       └── report_node.py
│
└── ghrs_bringup/            # Launch files & configuration
    ├── launch/
    │   ├── ghrs_full.launch.py
    │   └── ghrs_hardware.launch.py
    └── config/
        ├── ghrs_params.yaml
        └── waypoints_example.json
```

---
## GHRS Chassis and Body Showcase

![Demo](assets/Video 2026-05-31 22-48-20.mp4)



## Node Reference

**`motor_controller_node`** — Drives two BTS7960 H-bridge motor controllers via Raspberry Pi GPIO. Converts `geometry_msgs/Twist` into differential PWM. Includes a watchdog that stops motors if no command arrives within 500 ms.

**`servo_controller_node`** — Controls all PCA9685 servo channels. Accepts pulse-width (µs) or angle (degrees) per channel, enforcing hardware calibration limits. Centres all servos on startup and shutdown.

**`imu_node`** — Reads the BNO08x fusion IMU and publishes quaternion orientation plus a clean `0–360°` fused yaw heading.

**`gps_node`** — Parses NMEA sentences from the NEO-6M GPS and publishes `NavSatFix` with HDOP-estimated covariance.

**`camera_node`** — Captures from USB webcam , Publishes `sensor_msgs/Image`. Runs a dedicated MJPEG TCP server on port 5001 separate from ROS spin.

**`pump_led_node`** — Controls the water pump (GPIO 17) and status LED (GPIO 27), both active-low. Supports timed burst mode.

**`turret_tracking_node`** — Runs independent PID loops for turret pan and tilt to centre a detected plant target in frame. Includes a target-loss watchdog.

**`obstacle_detection_node`** — Camera-based obstacle detection using Canny edge density in the lower frame ROI, split into left/centre/right columns. Replaces LiDAR entirely.

**`plant_disease_node`** — Classifies plant health using a TFLite or PyTorch model when available, with a colour-deviation heuristic fallback. Saves timestamped evidence images and GPS-tags each detection.

**`navigation_node`** — GPS-guided autonomous patrol with IMU heading control. FSM: `IDLE → PATROL → AVOID → IDLE`. Triggers pesticide spray when diseased plant is confirmed.

**`pesticide_spray_node`** (`ghrs_fire` package) — Orchestrates the full spray sequence: pauses the rover, aims the turret at the target, activates the pump for a configurable duration, then resumes patrol.

**`report_node`** — Aggregates all mission events (plant disease detections, obstacle counts, GPS track distance) and generates a styled HTML report + JSON data file at mission end.

---

## Building

```bash
source /opt/ros/jazzy/setup.bash

# Messages must build first
colcon build --packages-select ghrs_msgs
source install/setup.bash

colcon build
source install/setup.bash
```

## Running

```bash
# Full system
ros2 launch ghrs_bringup ghrs_full.launch.py

# Hardware only (for testing)
ros2 launch ghrs_bringup ghrs_hardware.launch.py

# Start a patrol mission
ros2 service call /ghrs/start_mission std_srvs/srv/Trigger
ros2 service call /ghrs/nav_start std_srvs/srv/Trigger

# Emergency stop
ros2 service call /ghrs/estop std_srvs/srv/SetBool "{data: true}"

# Manual spray trigger
ros2 service call /ghrs/manual_spray std_srvs/srv/Trigger

# Generate mission report
ros2 service call /ghrs/end_mission std_srvs/srv/Trigger
```

---

<div align="center">
 
*Developer: Anmar Arafat Al-Momani*

</div>
