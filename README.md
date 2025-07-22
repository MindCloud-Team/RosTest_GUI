# RosTest_GUI
# Panther Robot Dashboard GUI

This repository contains a ROS 2-based GUI application for monitoring and controlling the **Panther robot**. Built using **PyQt5** and **VTK**, it visualizes real-time sensor data, camera feeds, and LiDAR point clouds, while also providing basic keyboard teleoperation.

##  Features

-  Live Battery status and voltage display
-  IMU orientation visualization (Roll, Pitch, Yaw)
-  Odometry position display
-  Emergency Stop (E-Stop) indicator
-  Live camera feeds (Front, Back, Left, Right)
-  LiDAR point cloud 3D visualization with interactive camera reset
-  Keyboard teleoperation (`W`, `A`, `S`, `D`)
-  Smooth and responsive GUI with multi-threaded image handling

##  Built With

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- Python 3
- [PyQt5](https://pypi.org/project/PyQt5/)
- [VTK](https://vtk.org/)
- OpenCV (`cv2`)
- `cv_bridge`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`

## Installation

### Dependencies

Install the required Python dependencies:

```bash
sudo apt install python3-pyqt5 python3-vtk ros-jazzy-cv-bridge
pip install opencv-python
