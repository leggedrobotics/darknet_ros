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
#include <pthread.h>
#include <thread>
#include <chrono>
#include <shared_mutex>
#include <numeric>
#include <queue>
#include <sstream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_transport/image_transport.h"

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include "darknet_ros_msgs/msg/object_count.hpp"
#include "darknet_ros_msgs/action/check_for_objects.hpp"

// Darknet.
#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "blas.h"
#include "image_opencv.h"

#include "image.h"
// #include "demo.h"
#include "darknet.h"

#include "image_interface.hpp"

#define START_COUNT 100

#include <sys/time.h>
}

extern "C" cv::Mat image_to_mat(image im);
extern "C" image mat_to_image(cv::Mat m);
// extern "C" void show_image(image p, const char* name);

// extern "C" void draw_detections_cv_v3(mat_cv* show_img, detection *dets, int num, float thresh, char **names, image **alphabet, int classes, int ext_output);

extern "C" void generate_image(image p, cv::Mat& disp);

namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct
{
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

typedef struct {
  cv::Mat image;
  std_msgs::msg::Header header;
} CvMatWithHeader_;

class YoloObjectDetector : public rclcpp::Node
{
 public:
  /*!
   * Constructor.
   */
  explicit YoloObjectDetector();

  /*!
   * Destructor.
   */
  ~YoloObjectDetector();

  /*!
   * Initialize the ROS connections.
   */
  void init();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Callback of camera.
   * @param[in] msg image pointer.
   */
  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);


  //! Typedefs.
  using CheckForObjectsAction = darknet_ros_msgs::action::CheckForObjects;
  using GoalHandleCheckForObjectsAction = rclcpp_action::ServerGoalHandle<CheckForObjectsAction>;

  /*!
   * Check for objects action goal callback.
   */
  rclcpp_action::GoalResponse checkForObjectsActionGoalCB(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CheckForObjectsAction::Goal> goal);

  /*!
   * Check for objects action preempt callback.
   */
  rclcpp_action::CancelResponse checkForObjectsActionPreemptCB(
    const std::shared_ptr<GoalHandleCheckForObjectsAction> goal_handle);

  /*!ºº
   * Check for objects action accept callback.
   */
  void checkForObjectsActionAcceptedCB(
    const std::shared_ptr<GoalHandleCheckForObjectsAction> goal_handle);

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

  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

  //! Check for objects action server.
  rclcpp_action::Server<CheckForObjectsAction>::SharedPtr checkForObjectsActionServer_;
  bool action_active_;
  bool preempt_requested_;
  std::shared_ptr<GoalHandleCheckForObjectsAction> goal_handle_;

  //! Advertise and subscribe to image topics.
  std::shared_ptr<image_transport::ImageTransport> it_;

  //! ROS subscriber and publisher.
  image_transport::Subscriber imageSubscriber_;
  rclcpp::Publisher<darknet_ros_msgs::msg::ObjectCount>::SharedPtr objectPublisher_;
  rclcpp::Publisher<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesPublisher_;

  //! Detected objects.
  std::vector<std::vector<RosBox_> > rosBoxes_;
  std::vector<int> rosBoxCounter_;
  darknet_ros_msgs::msg::BoundingBoxes boundingBoxesResults_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;

  //! Publisher of the bounding box image.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detectionImagePublisher_;

  // Yolo running on thread.
  std::thread yoloThread_;

  // Darknet.
  char **demoNames_;
  image **demoAlphabet_;
  int demoClasses_;

  std::string windowName_;

  network *net_;
  std_msgs::msg::Header headerBuff_[3];
  image buff_[3];
  image buffLetter_[3];
  int buffId_[3];
  int buffIndex_ = 0;
  // IplImage * ipl_;
  cv::Mat disp_;
  float fps_ = 0;
  float max_fps = 0;
  float min_fps = 10000;
  std::stringstream ss_fps;
  float demoThresh_ = 0;
  float demoHier_ = .5;
  int running_ = 0;

  int demoDelay_ = 0;
  int demoFrame_ = 3;
  float **predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float *lastAvg2_;
  float *lastAvg_;
  float *avg_;
  int demoTotal_ = 0;
  double demoTime_;

  RosBox_ *roiBoxes_;
  bool viewImage_;
  bool enableConsoleOutput_;
  int waitKeyDelay_;
  int fullScreen_;
  char *demoPrefix_;

  std_msgs::msg::Header imageHeader_;
  cv::Mat camImageCopy_;
  std::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  std::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  std::shared_mutex mutexNodeStatus_;

  int actionId_;
  std::shared_mutex mutexActionStatus_;

  // double getWallTime();

  int sizeNetwork(network *net);

  void rememberNetwork(network *net);

  detection *avgPredictions(network *net, int *nboxes);

  void *detectInThread();

  void *fetchInThread();

  void *displayInThread(void *ptr);

  void *displayLoop(void *ptr);

  void *detectLoop(void *ptr);

  void setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                    char **names, int classes,
                    int delay, char *prefix, int avg_frames, float hier, int w, int h,
                    int frames, int fullscreen);

  void yolo();

  // IplImageWithHeader_ getIplImageWithHeader();
  CvMatWithHeader_ getCvMatWithHeader();

  bool getImageStatus(void);

  bool isNodeRunning(void);

  void *publishInThread();
  void generate_image_cp(image p, cv::Mat& disp);
};

} /* namespace darknet_ros*/
