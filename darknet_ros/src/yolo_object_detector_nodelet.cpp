/*
 *   Author: Timon Homberger
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

class DarknetRosNodelet : public nodelet::Nodelet {
 public:
  DarknetRosNodelet() = default;
  ~DarknetRosNodelet() {
    if (darknetRos_) delete darknetRos_;
  }

 private:
  virtual void onInit() {
    ros::NodeHandle NodeHandle("~");
    NodeHandle = getPrivateNodeHandle();
    darknetRos_ = new darknet_ros::YoloObjectDetector(NodeHandle);
  }

  darknet_ros::YoloObjectDetector* darknetRos_;
};

// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(DarknetRosNodelet, nodelet::Nodelet);