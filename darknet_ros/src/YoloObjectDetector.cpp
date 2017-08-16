/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.h"

// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

cv::Mat camImageCopy_;
IplImage* get_ipl_image() {
  IplImage* ROS_img = new IplImage(camImageCopy_);
  return ROS_img;
}

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh):
     nodeHandle_(nh),
     imageTransport_(nodeHandle_),
     numClasses_(0),
     classLabels_(0),
     rosBoxes_(0),
     rosBoxCounter_(0),
     rosBoxColors_(0),
     opencvWindow_("YOLO V2 object detection")
{
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

bool YoloObjectDetector::readParameters() {
  // Load common parameters.
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/use_darknet", darknetImageViewer_, false);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  if(XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  }
  else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  if(!viewImage_) {
    darknetImageViewer_ = false;
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ =   std::vector< std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);
  rosBoxColors_ = std::vector<cv::Scalar>(numClasses_);

  return true;
}

void YoloObjectDetector::init() {
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize color of bounding boxes of different object classes.
  int incr = floor(255/numClasses_);
  for (int i = 0; i < numClasses_; i++) {
    rosBoxColors_[i] = cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i);
  }

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float)0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("tiny-yolo-voc.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("tiny-yolo-voc.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**)realloc((void*)detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  load_network_demo(cfg, weights, data,
                    thresh,
                    detectionNames, numClasses_,
                    darknetImageViewer_, waitKeyDelay_,
                    0,
                    0.5,
                    0, 0, 0, 0,
                    enableConsoleOutput_);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &YoloObjectDetector::cameraCallback,this);
  objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
  boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
  checkForObjectsActionServer_.reset(
      new CheckForObjectsActionServer(
          nodeHandle_, checkForObjectsActionName,
          false));
  checkForObjectsActionServer_->registerGoalCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();

  // OpenCv image view.
  if(viewImage_ && !darknetImageViewer_) {
    cv::namedWindow(opencvWindow_, cv::WINDOW_NORMAL);
    cv::moveWindow(opencvWindow_, 0, 0);
    cv::resizeWindow(opencvWindow_, 1352, 1013);
  }
}

YoloObjectDetector::~YoloObjectDetector() {
  if(viewImage_ && !darknetImageViewer_) {
    cv::destroyWindow(opencvWindow_);
  }
}

void YoloObjectDetector::drawBoxes(cv::Mat &inputFrame, std::vector<RosBox_> &rosBoxes, int &numberOfObjects,
   cv::Scalar &rosBoxColor, const std::string &objectLabel) {
  darknet_ros_msgs::BoundingBox boundingBox;

  for (int i = 0; i < numberOfObjects; i++) {
    int xmin = (rosBoxes[i].x - rosBoxes[i].w/2)*frameWidth_;
    int ymin = (rosBoxes[i].y - rosBoxes[i].h/2)*frameHeight_;
    int xmax = (rosBoxes[i].x + rosBoxes[i].w/2)*frameWidth_;
    int ymax = (rosBoxes[i].y + rosBoxes[i].h/2)*frameHeight_;

    boundingBox.Class = objectLabel;
    boundingBox.probability = rosBoxes[i].prob;
    boundingBox.xmin = xmin;
    boundingBox.ymin = ymin;
    boundingBox.xmax = xmax;
    boundingBox.ymax = ymax;
    boundingBoxesResults_.boundingBoxes.push_back(boundingBox);

    // draw bounding box of first object found
    cv::Point topLeftCorner = cv::Point(xmin, ymin);
    cv::Point botRightCorner = cv::Point(xmax, ymax);
    cv::rectangle(inputFrame, topLeftCorner, botRightCorner, rosBoxColor, 2);
    std::ostringstream probability;
    probability << rosBoxes[i].prob*100;
    cv::putText(inputFrame, objectLabel + " (" + probability.str() + "%)", cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
                1.0, rosBoxColor, 2.0);
  }
}

void YoloObjectDetector::runYolo(cv::Mat &fullFrame, int id) {
  if(enableConsoleOutput_) {
    ROS_INFO("[YoloObjectDetector] runYolo().");
  }

  cv::Mat inputFrame = fullFrame.clone();

  // run yolo and get bounding boxes for objects
  boxes_ = demo_yolo();

  // get the number of bounding boxes found
  int num = boxes_[0].num;

  // if at least one BoundingBox found, draw box
  if (num > 0  && num <= 100) {
    if(!darknetImageViewer_ && enableConsoleOutput_) {
      std::cout << "# Objects: " << num << std::endl;
    }
    // split bounding boxes by class
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
         if (boxes_[i].Class == j) {
            rosBoxes_[j].push_back(boxes_[i]);
            rosBoxCounter_[j]++;
            if(!darknetImageViewer_ && enableConsoleOutput_) {
              std::cout << classLabels_[boxes_[i].Class] << " (" << boxes_[i].prob*100 << "%)" << std::endl;
            }
         }
      }
    }

    // send message that an object has been detected
    std_msgs::Int8 msg;
    msg.data = 1;
    objectPublisher_.publish(msg);

    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) drawBoxes(inputFrame, rosBoxes_[i],
                                             rosBoxCounter_[i], rosBoxColors_[i], classLabels_[i]);
    }
    boundingBoxesPublisher_.publish(boundingBoxesResults_);
  }
  else {
    std_msgs::Int8 msg;
    msg.data = 0;
    objectPublisher_.publish(msg);
  }
  if (isCheckingForObjects()) {
    ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
    darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
    objectsActionResult.id = id;
    objectsActionResult.boundingBoxes = boundingBoxesResults_;
    checkForObjectsActionServer_->setSucceeded(objectsActionResult,"Send bounding boxes.");
  }
  boundingBoxesResults_.boundingBoxes.clear();

  for (int i = 0; i < numClasses_; i++) {
     rosBoxes_[i].clear();
     rosBoxCounter_[i] = 0;
  }

  if(viewImage_ && !darknetImageViewer_) {
    cv::imshow(opencvWindow_, inputFrame);
    cv::waitKey(waitKeyDelay_);
  }

  // Publish elevation change map.
  if (!publishDetectionImage(inputFrame)) ROS_DEBUG("Detection image has not been broadcasted.");
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(enableConsoleOutput_) {
    ROS_INFO("[YoloObjectDetector] USB image received.");
  }

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(cam_image) {
    camImageCopy_ = cam_image->image.clone();
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
    runYolo(cam_image->image);
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionGoalCB() {
  if(enableConsoleOutput_) {
    ROS_INFO("[YoloObjectDetector] Start check for objects action.");
  }

  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
  }

  if (cam_image) {
    camImageCopy_ = cam_image->image.clone();
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
    runYolo(cam_image->image, imageActionPtr->id);
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionPreemptCB() {
  ROS_INFO("[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const {
  return (ros::ok() &&
      checkForObjectsActionServer_->isActive() &&
          !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage) {
  if (detectionImagePublisher_.getNumSubscribers() < 1) return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image    = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

} /* namespace darknet_ros*/
