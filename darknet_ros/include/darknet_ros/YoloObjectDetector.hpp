/*
 * YoloObjectDetector.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// c++
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>

// Darknet.
#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

// Image interface.
#include "darknet_ros/image_interface.hpp"

extern "C" cv::Mat image_to_mat(image im);
extern "C" image mat_to_image(cv::Mat m);
extern "C" int show_image(image p, const char* name, int ms);

namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

typedef struct {
  cv::Mat image;
  std_msgs::Header header;
} CvMatWithHeader_;

class YoloObjectDetector {
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

  //! Using.
  using CheckForObjectsActionServer = actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction>;
  using CheckForObjectsActionServerPtr = std::shared_ptr<CheckForObjectsActionServer>;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

  //! Check for objects action server.
  CheckForObjectsActionServerPtr checkForObjectsActionServer_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;

  //! ROS subscriber and publisher.
  image_transport::Subscriber imageSubscriber_;
  ros::Publisher objectPublisher_;
  ros::Publisher boundingBoxesPublisher_;

  //! Detected objects.
  std::vector<std::vector<RosBox_> > rosBoxes_;
  std::vector<int> rosBoxCounter_;
  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;

  //! Publisher of the bounding box image.
  ros::Publisher detectionImagePublisher_;

  // Yolo running on thread.
  std::thread yoloThread_;

  // Darknet.
  char** demoNames_;
  image** demoAlphabet_;
  int demoClasses_;

  network* net_;
  std_msgs::Header headerBuff_[3];
  image buff_[3];
  image buffLetter_[3];
  int buffId_[3];
  int buffIndex_ = 0;
  float fps_ = 0;
  float demoThresh_ = 0;
  float demoHier_ = .5;
  int running_ = 0;
  cv::Mat disp_;
  int demoDelay_ = 0;
  int demoFrame_ = 3;
  float** predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float* lastAvg2_;
  float* lastAvg_;
  float* avg_;
  int demoTotal_ = 0;
  double demoTime_;

  RosBox_* roiBoxes_;
  bool viewImage_;
  bool enableConsoleOutput_;
  int waitKeyDelay_;
  int fullScreen_;
  char* demoPrefix_;

  std_msgs::Header imageHeader_;
  cv::Mat camImageCopy_;
  boost::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  boost::shared_mutex mutexNodeStatus_;

  int actionId_;
  boost::shared_mutex mutexActionStatus_;

  // double getWallTime();

  int sizeNetwork(network* net);

  void rememberNetwork(network* net);

  detection* avgPredictions(network* net, int* nboxes);

  void* detectInThread();

  void* fetchInThread();

  void* displayInThread(void* ptr);

  void* displayLoop(void* ptr);

  void* detectLoop(void* ptr);

  void setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay, char* prefix,
                    int avg_frames, float hier, int w, int h, int frames, int fullscreen);

  void yolo();

  CvMatWithHeader_ getCvMatWithHeader();

  bool getImageStatus(void);

  bool isNodeRunning(void);

  void* publishInThread();
};

} /* namespace darknet_ros*/
