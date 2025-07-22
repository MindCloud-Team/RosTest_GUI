# RosTestGUI

## Introduction

This is a Python-based GUI package designed for the Panther robot's dashboard in ROS 2. It integrates multiple components including video feeds, sensor data, and robot velocity control into a unified graphical interface.

##  Features

-  Battery status (percentage + voltage)
-  IMU orientation display (Roll, Pitch, Yaw)
-  Odometry position tracking
-  E-Stop indicator
-  Multi-camera feed viewer (Front, Back, Left, Right)
-  3D LiDAR point cloud visualization
-  Teleoperation using `W`, `A`, `S`, `D`
-  Responsive UI with threaded image processing

## Tech Stack

- ROS 2 Jazzy
- Python 3
- PyQt5
- VTK
- OpenCV (`cv2`)
- `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`

## Installation

### 1. Install Dependencies

```bash
sudo apt install python3-pyqt5 python3-vtk ros-jazzy-cv-bridge
pip install opencv-python
```

### 2. Clone & Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/ANO614/RosTest_GUI.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the GUI

Make sure your robot or simulation is publishing the required topics. Then launch the GUI with:

```bash
QT_QPA_PLATFORM=xcb python3 <full file destination>
```

> Ensure ROS 2 is properly sourced before launching.

## Teleop Keys

| Key | Action        |
|-----|---------------|
| W   | Move forward  |
| S   | Move backward |
| A   | Turn left     |
| D   | Turn right    |

## Screenshot / Demo

![Working GUI:](images/GUI.png)

## Folder Overview

```
gui_pkg/
├── batgui.py          # Main GUI entry point
├── resource/          # (Optional) assets
├── package.xml
├── setup.py
```

## Tested On

- Ubuntu 22.04
- ROS 2 Jazzy
- Panther robot (Husarion) — real & simulated

## Author

Developed by **MindCloud-Team**
