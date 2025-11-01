# Arm Trajectory Controller
Trajectory control packages for my robotic arm.

## Quickstart
Add the packages to your ROS2 workspace. Source and build the packages.
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source ./install/setup.bash
```

To control the individual joints with the joint state publisher launch file.
```bash
ros2 launch arm_description jsp_link.launch.py
```

To control the arm in cartesian space use the `control.launch.py` launch file.
```bash
ros2 launch arm_link control.launch.py
```

## Control Interface
To interface the with the trajectory controller


## Requirements
- Ubuntu 24.04+
- ROS2 Jazzy
- Jazzy control packages
- MoveIt!
