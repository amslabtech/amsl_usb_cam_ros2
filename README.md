# amsl_usb_cam_ros2
[![test](https://github.com/amslabtech/amsl_usb_cam_ros2/workflows/test/badge.svg)](https://github.com/amslabtech/amsl_usb_cam_ros2/actions)

ROS2 webcam driver

# Requirement

- ROS2 (tested on foxy) 

# Install and build
```
cd your_ros2_ws/src
git clone https://github.com/amslabtech/amsl_usb_cam_ros2.git
vcs install < amsl_usb_cam_ros2/.rosinstall 
cd your_ros2_ws
rosdep install -i -r -y --from-paths src
colcon build 
```

---
### *Note* 
Part of processes in this repo are implemented with reference to [image_tools](https://github.com/ros2/demos/tree/foxy/image_tools).
