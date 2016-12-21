/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include "darknet_rsl/YoloObjectDetector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_rsl");
  ros::NodeHandle nodeHandle("~");
  darknet_rsl::YoloObjectDetector yoloObjectDetector(nodeHandle);

  ros::spin();
  return 0;
}
