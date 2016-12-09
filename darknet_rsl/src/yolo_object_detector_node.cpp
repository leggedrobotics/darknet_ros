/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include "YoloObjectDetector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_interface");
  ros::NodeHandle nodeHandle("~");

  YoloObjectDetector yod(nodeHandle);
  ros::spin();
  return 0;
}
