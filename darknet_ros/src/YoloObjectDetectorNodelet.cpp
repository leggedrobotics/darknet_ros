/**
 * @file YoloObjectDetecorNodelet.cpp
 * @author Rhys McKercher (https://rhysmckercher@universalfieldrobots.com.au)
 * @brief Nodelet wrapper for YoloObjectDetector class.
 * @version 1.0
 * @date April-2020
 * 
 * @copyright Copyright (c) Universal Field Robots 2019
 * 
 */

#include <darknet_ros/YoloObjectDetectorNodelet.hpp>
#include <pluginlib/class_list_macros.h>

namespace darknet_ros
{

YoloObjectDetectorNodelet::YoloObjectDetectorNodelet():
  nh_("darknet_ros")  // YoloObjectDetector(nh_)call super constructor for base class
{
  yolo_object_detector = std::unique_ptr<YoloObjectDetector>(new YoloObjectDetector(nh_));
}


YoloObjectDetectorNodelet::~YoloObjectDetectorNodelet()
{
  yolo_object_detector.reset(nullptr);
}

void YoloObjectDetectorNodelet::onInit()
{
  ROS_INFO("[YoloObjectDetectorNodelet] ~~~~~~~ Nodelet Initalised ~~~~~~");
}

}

PLUGINLIB_EXPORT_CLASS(darknet_ros::YoloObjectDetectorNodelet, nodelet::Nodelet)