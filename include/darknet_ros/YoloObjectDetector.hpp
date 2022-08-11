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

// UML
#include <umd_rtsp/rtsp_streamer.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// C++
#include <iostream>
#include <string>

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
#include "darknet.h"

#include "image_interface.hpp"

#define START_COUNT 100

#include <sys/time.h>
}

extern "C" cv::Mat image_to_mat(image im);
extern "C" image mat_to_image(cv::Mat m);
extern "C" void generate_image(image p, cv::Mat& disp);

namespace darknet_ros {
namespace utility
{
/**
 * @brief Retrieve the value of an environment variable
 * safely. Return 'true' if successful, or 'false' if not.
 * NOTE: The value of 'var' will not be modified if it is
 * not possible to read the environment variable.
 *
 * @param name Name of the environment variable
 * @param var Value of the environment variable
 * @return true
 * @return false
 */
bool
safe_getenv(const std::string& name, std::string& var);
}  // namespace utility

//! Bounding box of the detected object.
// typedef struct
// {
//   float x, y, w, h, prob;
//   int num, Class;
// } RosBox_;

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

  void on_image_callback(const cv::Mat& image);


  //! ROS publisher.
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

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

  network *net_;
  image buff_[3];
  image buffLetter_[3];
  int buffId_[3];
  int buffIndex_ = 0;
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
  // float **predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float *avg_;
  int demoTotal_ = 0;
  double demoTime_;

  umd_rtsp::RTSPStreamer rtsp_streamer_;

  char *demoPrefix_;

  cv::Mat camImageCopy_;
  std::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  std::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  std::shared_mutex mutexNodeStatus_;

  int sizeNetwork(network *net);

  void rememberNetwork(network *net);

  detection *avgPredictions(network *net, int *nboxes);

  void *detectInThread();

  void *fetchInThread();

  void setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                    char **names, int classes,
                    int delay, char *prefix, int avg_frames, float hier, int w, int h,
                    int frames);

  void yolo();

  bool getImageStatus(void);

  bool isNodeRunning(void);

  void generate_image_cp(image p, cv::Mat& disp);
};

} /* namespace darknet_ros*/