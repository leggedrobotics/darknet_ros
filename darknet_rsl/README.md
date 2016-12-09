#Darknet#
Darknet is an open source neural network framework written in C and CUDA. It is fast, easy to install, and supports CPU and GPU computation.

For more information see the [Darknet project website](http://pjreddie.com/darknet).

For questions or issues please use the [Google Group](https://groups.google.com/forum/#!forum/darknet).

#Darknet ROS#
This version of darknet provides an ROS interface for running the YOLO detection as an ROS node.  The default config uses the pascal VOC detection weights but this interface can be used with any custom weights.

ros_interface.cpp replaces the darknet.c executable and acts as the ROS entry point.  It includes a subscriber to a /usb_cam image topic and a function that sends that image to the YOLO source code.  yolo_kernels.cu has been modified to receive images from ros_interface.cpp rather than from CvVideoCapture.

To use: Modify ros_interface.cpp with the correct path to your yolo-tiny.weights and change the /usb_cam/image_raw topic to your camera topic.  Compile normally with catkin_make and run with "rosrun darknet_ros ros_interface".

NEW: YoloObjectDetector.cpp gives you full control of the output of YOLO.  This ROS node extracts the bounding box coordinates from the YOLO source code and annotates the images itself.  It publishes two topics: /found_object displays "1" or "0" corresponding to whether or not an object has been detected, and /YOLO_bboxes displays the class label that was detected followed by the bbox coordinates [xmin, ymin, xmax, ymax]. 

To use: Modify YoloObjectDetector.cpp with the correct path to your yolo-tiny.weights and yolo-tiny.cfg file, change the /usb_cam/image_raw topic to your camera topic, and change the class_labels[] array with your desired labels.  Run with "rosrun darknet_ros yolo_object_detector".


