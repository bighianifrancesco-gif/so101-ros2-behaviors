# SO-101 ROS 2 Behavior Control

Behavior-based control system for the **SO-101 robotic arm** using **ROS 2** and **IMU-driven interaction**.

## Overview

This project was developed as a hands-on experiment during a robotics internship to explore system integration, real-time control, and interactive robot behavior.

The robot reacts to physical interaction: when the arm is pushed, an IMU mounted on the end-effector detects both the disturbance and its direction, triggering different expressive behaviors.

The focus is on building a simple but complete pipeline:

**sensing → processing → behavior selection → motion execution**

## Features

- ROS 2 control architecture for the SO-101 robotic arm
- IMU-based push detection (MPU6050)
- Direction-aware interaction (left/right detection)
- Multiple behavior modes:
  - **disappointed** — expressive “no” gesture
  - **scared** — fast recoil + slow recovery
  - **angry** — aggressive “bite” behavior toward the push direction
- Smooth motion using interpolated joint trajectories
- RViz digital twin for visualization and testing

## System Architecture

- **Hardware**
  - SO-101 robotic arm (Feetech servos)
  - MPU6050 IMU (Arduino-based serial interface)

- **ROS 2 Nodes**
  - `so101_hardware_py`
    - arm interface (motor control + feedback)
    - IMU serial node
  - `so101_control_py`
    - behavior controller (state machine + reactive behaviors)
  - `so101_description`
    - URDF + RViz visualization
  - `so101_bringup`
    - integration / launch utilities

## Behavior Logic

- IMU measures orientation changes (roll/pitch)
- A push is detected from roll variation
- The sign of the roll determines the interaction side
- A state machine triggers a behavior sequence
- Motion is generated via smooth interpolation between poses

## Installation

Requirements:
- Ubuntu 22.04
- ROS 2 Humble
- Python 3

Clone the repository inside your ROS 2 workspace:

``bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/so101-ros2-behaviors.git

Build:

cd ~/ros2_ws
colcon build
source install/setup.bash
Usage

Run nodes (example):

ros2 run so101_hardware_py arm_interface_node
ros2 run so101_hardware_py imu_serial_node
ros2 run so101_control_py behaviour_controller_node

Switch behavior mode live:

ros2 param set /behavior_controller_node mode disappointed
ros2 param set /behavior_controller_node mode scared
ros2 param set /behavior_controller_node mode angry
Demo

Short demo video:
👉 (add your LinkedIn post or video link here)

Notes
This project is designed as a compact integration demo rather than a full production system
Some parameters (e.g. calibration files or device ports) may need to be adapted to your setup
Serial devices are recommended to be accessed via /dev/serial/by-id/... for stability
Future Improvements
Add additional behavior modes
Use push intensity to scale response
Improve motion realism (velocity/acceleration profiles)
Integrate compliant or soft-actuated elements
Author

Francesco Bighiani
Master’s student in Mechatronics Engineering — Politecnico di Torino
