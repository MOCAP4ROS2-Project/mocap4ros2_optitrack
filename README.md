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
