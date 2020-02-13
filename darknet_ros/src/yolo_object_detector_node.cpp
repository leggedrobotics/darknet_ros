/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/YoloObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto yoloObjectDetector = std::make_shared<darknet_ros::YoloObjectDetector>();

  yoloObjectDetector->init();
  
  rclcpp::spin(yoloObjectDetector->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
