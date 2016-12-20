/*
 * YoloObjectDetector.h
 *
 *  Created on: Dec 08, 2016
 *      Author: Marko, Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "ros_interface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_rsl_msgs/BoundingBoxes.h>
#include <darknet_rsl_msgs/BoundingBox.h>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <darknet_rsl_msgs/CheckForObjectsAction.h>

extern "C" {
  #include "box.h"
}

class YoloObjectDetector
{
 public:
  /*!
   * Constructor.
   */
  YoloObjectDetector(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~YoloObjectDetector();

 private:

  /*!
   * Draws the bounding boxes of the detected objects.
   * @param[in] inputFrame image of current camera frame.
   * @param[in] rosBoxes vector of detected bounding boxes of specific class.
   * @param[in] numberOfObjects number of objects of specific class.
   * @param[in] rosBoxColor color of specific class.
   * @param[in] objectLabel name of detected class.
   */
  void drawBoxes(cv::Mat &inputFrame, std::vector<RosBox_> &rosBoxes, int &numberOfObjects,
      cv::Scalar &rosBoxColor, const std::string &objectLabel);

  /*!
   * Run YOLO and detect obstacles.
   * @param[in] fullFrame image of current camera frame.
   */
  void runYolo(cv::Mat &fullFrame);

  /*!
   * Callback of camera.
   * @param[in] msg image pointer.
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * Check for objects action goal callback.
   */
  void checkForObjectsActionGoalCB();

  /*!
   * Check for objects action preempt callback.
   */
  void checkForObjectsActionPreemptCB();

  /*!
   * Check if a preempt for the check for objects action has been requested.
   * @return false if preempt has been requested or inactive.
   */
  bool isCheckingForObjects() const;

  //! Typedefs.
  typedef actionlib::SimpleActionServer<darknet_rsl_msgs::CheckForObjectsAction> CheckForObjectsActionServer;
  typedef std::shared_ptr<CheckForObjectsActionServer> CheckForObjectsActionServerPtr;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! Check for objects action server.
  CheckForObjectsActionServerPtr checkForObjectsActionServer_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;

  //! ROS subscriber and publisher.
  image_transport::Subscriber imageSubscriber_;
  ros::Publisher objectPublisher_;
  ros::Publisher boundingBoxesPublisher_;

  //! Detected objects.
  std::vector< std::vector<RosBox_> > rosBoxes_;
  std::vector<int> rosBoxCounter_;
  std::vector<cv::Scalar> rosBoxColors_;
  darknet_rsl_msgs::BoundingBoxes boundingBoxesResults_;
  RosBox_* boxes_;

  //! Camera related parameters.
  std::string cameraTopicName_;
  const std::string opencvWindow_;
  int frameWidth_;
  int frameHeight_;
};
