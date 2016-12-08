/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include "yolo_object_detector.h"

extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_interface");

  std::string param;
  float thresh;
  std::string cameraTopicName;
  ros::param::get("/darknet_rsl/object_threshold", thresh);
  ros::param::get("/darknet_rsl/camera_topic_name", cameraTopicName);
  ros::param::get("/darknet_rsl/weights_path", param);
  char *weights = new char[param.length() + 1];
  strcpy(weights, param.c_str());
  ros::param::get("/darknet_rsl/cfg_path", param);
  char *cfg = new char[param.length() + 1];
  strcpy(cfg, param.c_str());

  load_network(cfg, weights, thresh);

  YoloObjectDetector yod(cameraTopicName);
  ros::spin();
  return 0;
}
