# Multi-class object detection with darknet/yolo

## Introduction
This is a ROS node for muli-class object detection using darknet/yolo Deep Neural Network (DNN).


## Compilation

Fully check out this repository with submodule ```darknet```. Compile the ```darknet``` library with the following commands:

```
cd darknet;mkdir build;cd build
cmake -DCMAKE_INSTALL_PREFIX=<full path to dn_object_detect> ..
make install
```

Go back to the catkin workspace base directory and compile the ROS node with

```
catkin_make dn_object_detect
```

## Running Node

**NOTE** You need to download the model/weights file separately.

```roslaunch dn_object_detect objdetect.launch```

The node will publish the following two topics

```/dn_object_detect/detected_objects```  detected object list.

```/dn_object_detect/debug_view``` debugging image stream.
