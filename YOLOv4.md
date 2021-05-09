# Darknet ROS + YOLO v4 + ROS-Foxy

Writer: [Ar-Ray](https://github.com/Ar-Ray-code)

## Requirements

- ROS-Foxy
- CUDA 11.2
- OpenCV4



## Installation

```bash
$ cd
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone --branch foxy-v4 --recursive https://github.com/Ar-Ray-code/darknet_ros.git
$ source /opt/ros/foxy/setup.bash
$ cd ~/ros2_ws
$ colcon build --symlink-install
```



## Execute YOLO v4

Terminal 1

```bash
$ source /opt/ros/foxy/setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera/rgb
```

Terminal 2

```bash
$ source /opt/ros/foxy/setup.bash
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run darknet_ros yolov4.launch.py
```

