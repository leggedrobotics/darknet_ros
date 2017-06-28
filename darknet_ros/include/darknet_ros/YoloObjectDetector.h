/*
 * YoloObjectDetector.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <pthread.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

/*!
 * Run YOLO and detect obstacles.
 * @param[out] bounding box.
 */
extern "C" RosBox_ *demo_yolo();

/*!
 * Initialize darknet network of yolo.
 * @param[in] cfgfile location of darknet's cfg file describing the layers of the network.
 * @param[in] weightfile location of darknet's weights file setting the weights of the network.
 * @param[in] datafile location of darknet's data file.
 * @param[in] thresh threshold of the object detection (0 < thresh < 1).
 */
extern "C" void load_network_demo(char *cfgfile, char *weightfile, char *namefile, char *datafile,
                       float thresh, int numClasses,
                       bool viewimage, int waitkeydelay,
                       int frame_skip,
                       float hier,
                       int w, int h, int frames, int fullscreen);

/*!
 * This function is called in yolo and allows YOLO to receive the ROS image.
 * @param[out] current image of the camera.
 */
IplImage* get_ipl_image(void);

//! Class labels.
const std::string classLabels_[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
    "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
    "potted plant", "sheep", "sofa", "train", "tv monitor" };
const int numClasses_ = sizeof(classLabels_)/sizeof(classLabels_[0]);

class YoloObjectDetector
{
 public:
  /*!
   * Constructor.
   */
  explicit YoloObjectDetector(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~YoloObjectDetector();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialize the ROS connections.
   */
  void init();

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
  void runYolo(cv::Mat &fullFrame, int id = 0);

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

  /*!
   * Publishes the detection image.
   * @return true if successful.
   */
  bool publishDetectionImage(const cv::Mat& detectionImage);

  //! Typedefs.
  typedef actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction> CheckForObjectsActionServer;
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
  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;
  RosBox_* boxes_;

  //! Camera related parameters.
  std::string cameraTopicName_;
  int frameWidth_;
  int frameHeight_;

  //! Image view in opencv.
  const std::string opencvWindow_;
  bool viewImage_;
  int waitKeyDelay_;
  bool darknetImageViewer_;

  //! Publisher of the bounding box image.
  ros::Publisher detectionImagePublisher_;
};

} /* namespace darknet_ros*/
