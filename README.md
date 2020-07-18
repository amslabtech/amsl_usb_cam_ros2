# amsl_usb_cam_ros2
ROS2 webcam driver

# Requirement

- ROS2 (tested on foxy) 

# Install and build
```
cd your_ros2_ws/src
git clone https://github.com/amslabtech/obstacle_point_cloud_detector.git
vcs install < obstacle_point_cloud_detector/.rosinstall 
cd your_ros2_ws
rosdep install -i -r -y --from-paths src
colcon build 
```

---
### *Note* 
Part of processes in this repo are implemented with reference to [image_tools](https://github.com/ros2/demos/tree/foxy/image_tools).
