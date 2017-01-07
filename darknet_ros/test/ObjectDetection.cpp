/*
 * ObjectDetection.cpp
 *
 *  Created on: Jan 07, 2017
 *      Author: Marko Bjelonic
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */


// Google Test
#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <actionlib/client/simple_action_client.h>

// boost
#include <boost/thread.hpp>

// OpenCV2.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// Actions.
#include <darknet_ros_msgs/CheckForObjectsAction.h>

// param io
#include <param_io/get_param.hpp>

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> CheckForObjectsActionClient;
typedef std::shared_ptr<CheckForObjectsActionClient> CheckForObjectsActionClientPtr;

// c++
#include <string>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

/*!
 * Done-callback for CheckForObjects action client.
 * @param[in] state
 * @param[in] result
 */
void checkForObjectsResultCB(
    const actionlib::SimpleClientGoalState& state,
    const darknet_ros_msgs::CheckForObjectsResultConstPtr& result)
{
  ROS_INFO("[ObjectDetectionTest] Received bounding boxes.");

  boundingBoxesResults_ = result->boundingBoxes;
}

void sendImageToYolo(ros::NodeHandle nh, std::string imageName)
{
  //!Check for objects action client.
  CheckForObjectsActionClientPtr checkForObjectsActionClient;

  // Action clients.
  checkForObjectsActionClient.reset(
      new CheckForObjectsActionClient(
          nh, param_io::getParam<std::string>(nh, "/darknet_ros/camera_action"),
          false));

  // Wait till action server launches.
  checkForObjectsActionClient->waitForServer(ros::Duration(2.0));

  // Path to test image.
  std::string pathToTestImage = darknetFilePath_;
  pathToTestImage += "/data/";
  pathToTestImage += imageName;
  pathToTestImage += ".jpg";

  // Get test image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
  cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

  // Generate goal.
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.image = *image;

  // Send goal.
  checkForObjectsActionClient->sendGoal(
      goal,
      boost::bind(&checkForObjectsResultCB, _1, _2),
      CheckForObjectsActionClient::SimpleActiveCallback(),
      CheckForObjectsActionClient::SimpleFeedbackCallback());

  checkForObjectsActionClient->waitForResult(ros::Duration(10.0));
}

TEST(ObjectDetection, DetectDog)
{
  srand((unsigned int) time(0));
  ros::NodeHandle nodeHandle("~");

  sendImageToYolo(nodeHandle, "dog");
}

TEST(ObjectDetection, DetectPerson)
{
  srand((unsigned int) time(0));
  ros::NodeHandle nodeHandle("~");

  sendImageToYolo(nodeHandle, "person");
}
