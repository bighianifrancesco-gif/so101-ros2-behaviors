# SO-101 ROS 2 Behavior Control

Behavior-based control system for the **SO-101 robotic arm** using **ROS 2** and **IMU-driven interaction**, with an **RViz digital twin** for visualization and testing.

---

## Overview

This project was developed as a hands-on experiment during a robotics internship to explore system integration, real-time control, and interactive robot behavior.

The robot reacts to physical interaction: when the arm is pushed, an IMU mounted on the end-effector detects both the disturbance and its direction, triggering different expressive behaviors.

The system implements a simple but complete pipeline:

**sensing → processing → behavior selection → motion execution**

---

## Features

- ROS 2 control architecture for the SO-101 robotic arm  
- IMU-based push detection (MPU6050)  
- Direction-aware interaction (left/right detection)  
- Multiple behavior modes:
  - **disappointed** — expressive “no” gesture  
  - **scared** — fast recoil + slow recovery  
  - **angry** — aggressive “bite” behavior  
- Smooth motion via interpolated joint trajectories  
- RViz digital twin for visualization and debugging  

---

## System Architecture

### Hardware
- SO-101 robotic arm (Feetech servos)  
- MPU6050 IMU (Arduino serial interface)  

### ROS 2 Packages

- `so101_hardware_py`  
  - arm interface (motor control + feedback)  
  - IMU serial node  

- `so101_control_py`  
  - behavior controller (state machine + reactive behaviors)  

- `so101_description`  
  - URDF model + RViz visualization  

- `so101_bringup`  
  - integration / launch utilities  

---

## Installation

### Requirements

- Ubuntu 22.04  
- ROS 2 Humble  
- Python 3  

### Clone repository

``bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/so101-ros2-behaviors.git
Build workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
Usage
1. Start hardware interface
ros2 run so101_hardware_py arm_interface_node
2. Start IMU node
ros2 run so101_hardware_py imu_serial_node
3. Start behavior controller
ros2 run so101_control_py behaviour_controller_node
4. Launch RViz (digital twin)
ros2 launch so101_description display.launch.py

This allows visualization of the robot and helps debugging the behavior in real time.

5. Switch behavior mode (live)
ros2 param set /behavior_controller_node mode disappointed
ros2 param set /behavior_controller_node mode scared
ros2 param set /behavior_controller_node mode angry
Demo

👉 (https://www.linkedin.com/feed/update/urn:li:activity:7444759246388461568/?originTrackingId=2RtYBHH2mDHeswThPSEr8w%3D%3D)

Notes
This project is intended as a compact integration demo
Some parameters (serial ports, calibration files) must be adapted to your setup
For stable USB communication, use:
/dev/serial/by-id/...

instead of /dev/ttyUSB0

Future Improvements
Additional behavior modes
Push intensity → adaptive response
More realistic motion profiles
Integration with compliant or soft-actuated systems
Author

Francesco Bighiani
Master’s student in Mechatronics Engineering — Politecnico di Torino
