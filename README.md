## Fork of [mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack) used for the [ROSbloX](https://rosblox.github.io/) project. 

Includes these changes:
- Adds an option to autostart the nodes
- Doesn't include this issue (at least for us, not investigated why): https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/issues/18
- Compensates for system delays of the Optitrack System using https://docs.optitrack.com/developer-tools/natnet-sdk/latency-measurements, see image

![image](https://github.com/rosblox/mocap4ros2_optitrack/assets/20051567/ebed2352-8470-46e9-b26b-b454896da336)


# mocap_optitrack_driver

[![GitHub Action Status](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/main.yaml/badge.svg)](https://github.com/MOCAP4ROS2-Project/mocap_optitrack_driver)

[![codecov](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver/main/graph/badge.svg)](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver)

Create workspace:
```
mkdir -p mocap_ws/src && cd mocap_ws/src
```
Download optitrack repo:
```
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
```
Install dependencies:
```
vcs import < mocap4ros2_optitrack/dependency_repos.repos
```
Compiling workspace:
```
cd .. && colcon build --symlink-install
```
Source workspace:
```
source install/setup.bash
```
Setup your optitrack configuration:
```
mocap_ws/src/mocap4ros2_optitrack/mocap_optitrack_driver/config/mocap_optitrack_driver_params.yaml
```
Launch optitrack system:
```
ros2 launch mocap_optitrack_driver optitrack2.launch.py
```
Check that Optitrack configuration works fine and is connected. As the driver node is a lifecycle node, you should transition to activate:
```
ros2 lifecycle set /mocap_optitrack_driver_node activate
```
Visualize in rViz:
```
ros2 launch mocap_marker_viz mocap_marker_viz.launch.py mocap_system:=optitrack
```
