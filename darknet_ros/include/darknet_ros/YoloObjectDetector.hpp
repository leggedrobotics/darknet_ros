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
#include "darknet_ros/image_interface.h"
#include <sys/time.h>
}

extern "C" void ipl_into_image(IplImage* src, image im);
extern "C" image ipl_to_image(IplImage* src);
extern "C" void convert_yolo_detections(float *predictions, int classes, int num, int square,
                                        int side, int w, int h, float thresh, float **probs,
                                        box *boxes, int only_objectness);
extern "C" void draw_yolo(image im, int num, float thresh, box *boxes, float **probs);
extern "C" void show_image_cv(image p, const char *name, IplImage *disp);

namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct
{
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

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
  void runYolo(cv::Mat &fullFrame, const std_msgs::Header& header, int id = 0);

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
  std::vector<cv::Scalar> rosBoxColors_;
  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;
  RosBox_* boxes_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;

  //! Image view in opencv.
  const std::string opencvWindow_;
  bool viewImage_;
  int waitKeyDelay_;
  bool darknetImageViewer_;
  bool enableConsoleOutput_;

  //! Publisher of the bounding box image.
  ros::Publisher detectionImagePublisher_;

  // Yolo running on thread.
  std::thread yoloThread_;

  // Darknet.
  char **demo_names;
  image **demo_alphabet;
  int demo_classes;

  float **probs;
  box *boxes;
  network net;
  image buff[3];
  image buff_letter[3];
  int buff_index = 0;
  IplImage * ipl;
  float fps = 0;
  float demo_thresh = 0;
  float demo_hier = .5;
  int running = 0;

  int demo_delay = 0;
  int demo_frame = 3;
  int demo_detections = 0;
  float **predictions;
  int demo_index = 0;
  int demo_done = 0;
  float *last_avg2;
  float *last_avg;
  float *avg;
  double demo_time;

  RosBox_ *ROI_boxes;
  bool view_image;
  bool enable_console_output;
  int wait_key_delay;
  int full_screen;
  char *demo_prefix;

  cv::Mat camImageCopy_;
  boost::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  boost::shared_mutex mutexNodeStatus_;


  double get_wall_time();

  void *fetch_in_thread();

  void *detect_in_thread();

  void *display_in_thread(void *ptr);

  void *display_loop(void *ptr);

  void *detect_loop(void *ptr);

  void setup_network(char *cfgfile, char *weightfile, char *datafile, float thresh,
                     char **names, int classes, bool viewimage, int waitkeydelay,
                     int delay, char *prefix, int avg_frames, float hier, int w, int h,
                     int frames, int fullscreen, bool enableConsoleOutput);

  void yolo();

  IplImage* get_ipl_image();

  bool get_image_status(void);

  bool is_node_running(void);

  void *publish_in_thread();
};

} /* namespace darknet_ros*/
