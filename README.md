# mocap4r2_optitrack_driver

[![rolling](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/rolling.yaml/badge.svg)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/rolling.yaml)

[![codecov](https://codecov.io/gh/MOCAP4ROS2-Project/mocap4r2_optitrack_driver/rolling/graph/badge.svg)](https://codecov.io/gh/MOCAP4ROS2-Project/mocap4r2_optitrack_driver)

Create workspace:
```
mkdir -p mocap4r2_ws/src && cd mocap4r2_ws/src
```
Download optitrack repo:
```
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
```
Install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
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
mocap4r2_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
```
Launch optitrack system:
```
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```
Check that Optitrack configuration works fine and is connected. As the driver node is a lifecycle node, you should transition to activate:
```
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
Visualize in rViz:
```
ros2 launch mocap4r2_marker_viz mocap4r2_marker_viz.launch.py mocap4r2_system:=optitrack
```
