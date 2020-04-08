
/**
 * @file YoloObjectDetecorNodelet.hpp
 * @author Rhys McKercher (https://rhysmckercher@universalfieldrobots.com.au)
 * @brief Nodelet wrapper for YoloObjectDetector class.
 * @version 1.0
 * @date April-2020
 * 
 * @copyright Copyright (c) Universal Field Robots 2019
 * 
 */

#pragma once

#include <darknet_ros/YoloObjectDetector.hpp>
#include <nodelet/nodelet.h>

namespace darknet_ros
{

class YoloObjectDetectorNodelet : public nodelet::Nodelet
{
public:

  YoloObjectDetectorNodelet();

  ~YoloObjectDetectorNodelet();

  void onInit();

private:

  ros::NodeHandle nh_;
  std::unique_ptr<YoloObjectDetector> yolo_object_detector;

};

}