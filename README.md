# Panther Robot Dashboard GUI

This repository provides a GUI dashboard for the **Husarion Panther robot**, built using **ROS 2 Jazzy**, **PyQt5**, and **VTK**. The dashboard displays real-time sensor data, camera feeds, LiDAR point cloud, and supports keyboard teleoperation.

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
ros2 run gui_pkg GUI
```

Or:

```bash
python3 src/gui_pkg/gui_pkg/GUI.py
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

> *(images showing the GUI)*

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
