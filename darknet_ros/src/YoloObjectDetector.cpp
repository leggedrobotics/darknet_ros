/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

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

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh),
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

YoloObjectDetector::~YoloObjectDetector()
{
  if (viewImage_ && !darknetImageViewer_) {
    cv::destroyWindow(opencvWindow_);
  }
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Load common parameters.
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/use_darknet", darknetImageViewer_, false);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  if (!viewImage_) {
    darknetImageViewer_ = false;
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);
  rosBoxColors_ = std::vector<cv::Scalar>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize color of bounding boxes of different object classes.
  int incr = floor(255 / numClasses_);
  for (int i = 0; i < numClasses_; i++) {
    rosBoxColors_[i] = cv::Scalar(255 - incr * i, 0 + incr * i, 255 - incr * i);
  }

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                    std::string("tiny-yolo-voc.weights"));
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
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setup_network(cfg, weights, data, thresh, detectionNames, numClasses_, darknetImageViewer_,
                waitKeyDelay_, 0, 0, 1, 0.5, 0, 0, 0, 0, enableConsoleOutput_);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

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

  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName,
                    std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                    std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize,
                                               &YoloObjectDetector::cameraCallback, this);
  objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName,
                                                           objectDetectorQueueSize,
                                                           objectDetectorLatch);
  boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
      boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                                                       detectionImageQueueSize,
                                                                       detectionImageLatch);

  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName,
                    std::string("check_for_objects"));
  checkForObjectsActionServer_.reset(
      new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();

  // OpenCv image view.
  if (viewImage_ && !darknetImageViewer_) {
    cv::namedWindow(opencvWindow_, cv::WINDOW_NORMAL);
    cv::moveWindow(opencvWindow_, 0, 0);
    cv::resizeWindow(opencvWindow_, 1352, 1013);
  }
}

void YoloObjectDetector::drawBoxes(cv::Mat &inputFrame, std::vector<RosBox_> &rosBoxes,
                                   int &numberOfObjects, cv::Scalar &rosBoxColor,
                                   const std::string &objectLabel)
{
  darknet_ros_msgs::BoundingBox boundingBox;

  for (int i = 0; i < numberOfObjects; i++) {
    int xmin = (rosBoxes[i].x - rosBoxes[i].w / 2) * frameWidth_;
    int ymin = (rosBoxes[i].y - rosBoxes[i].h / 2) * frameHeight_;
    int xmax = (rosBoxes[i].x + rosBoxes[i].w / 2) * frameWidth_;
    int ymax = (rosBoxes[i].y + rosBoxes[i].h / 2) * frameHeight_;

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
    probability << rosBoxes[i].prob * 100;
    cv::putText(inputFrame, objectLabel + " (" + probability.str() + "%)",
                cv::Point(xmin, ymax + 15), cv::FONT_HERSHEY_PLAIN, 1.0, rosBoxColor, 2.0);
  }
}

void YoloObjectDetector::runYolo(cv::Mat &fullFrame, const std_msgs::Header& header, int id)
{
  ROS_DEBUG("[YoloObjectDetector] runYolo().");

  cv::Mat inputFrame = fullFrame.clone();

  // run yolo and get bounding boxes for objects
//  boxes_ = demo_yolo();

// get the number of bounding boxes found
  int num = boxes_[0].num;

  // if at least one BoundingBox found, draw box
  if (num > 0 && num <= 100) {
    if (!darknetImageViewer_ && enableConsoleOutput_) {
      std::cout << "# Objects: " << num << std::endl;
    }
    // split bounding boxes by class
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (boxes_[i].Class == j) {
          rosBoxes_[j].push_back(boxes_[i]);
          rosBoxCounter_[j]++;
          if (!darknetImageViewer_ && enableConsoleOutput_) {
            std::cout << classLabels_[boxes_[i].Class] << " (" << boxes_[i].prob * 100 << "%)"
                      << std::endl;
          }
        }
      }
    }

    // send message that an object has been detected
    std_msgs::Int8 msg;
    msg.data = num;
    objectPublisher_.publish(msg);

    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0)
        drawBoxes(inputFrame, rosBoxes_[i], rosBoxCounter_[i], rosBoxColors_[i], classLabels_[i]);
    }
    boundingBoxesResults_.header = header;
    boundingBoxesPublisher_.publish(boundingBoxesResults_);
  } else {
    std_msgs::Int8 msg;
    msg.data = 0;
    objectPublisher_.publish(msg);
  }
  if (isCheckingForObjects()) {
    ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
    darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
    objectsActionResult.id = id;
    objectsActionResult.boundingBoxes = boundingBoxesResults_;
    checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
  }
  boundingBoxesResults_.boundingBoxes.clear();

  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  if (viewImage_ && !darknetImageViewer_) {
    cv::imshow(opencvWindow_, inputFrame);
    cv::waitKey(waitKeyDelay_);
  }

  // Publish detection image.
  if (!publishDetectionImage(inputFrame))
    ROS_DEBUG("Detection image has not been broadcasted.");
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[YoloObjectDetector] USB image received.");

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionGoalCB()
{
  ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr =
      checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    camImageCopy_ = cam_image->image.clone();
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
    runYolo(cam_image->image, imageAction.header, imageActionPtr->id);
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionPreemptCB()
{
  ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const
{
  return (ros::ok() && checkForObjectsActionServer_->isActive()
      && !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
{
  if (detectionImagePublisher_.getNumSubscribers() < 1)
    return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

double YoloObjectDetector::get_wall_time()
{
  struct timeval time;
  if (gettimeofday(&time, NULL)) {
    return 0;
  }
  return (double) time.tv_sec + (double) time.tv_usec * .000001;
}

void *YoloObjectDetector::fetch_in_thread()
{
  IplImage* ROS_img = get_ipl_image();
  ipl_into_image(ROS_img, buff[buff_index]);
  rgbgr_image(buff[buff_index]);
//  delete ROS_img;
//  ROS_img = NULL;
  letterbox_image_into(buff[buff_index], net.w, net.h, buff_letter[buff_index]);
  return 0;
}

void *YoloObjectDetector::detect_in_thread()
{
  running = 1;
  float nms = .4;

  layer l = net.layers[net.n - 1];
  float *X = buff_letter[(buff_index + 2) % 3].data;
  float *prediction = network_predict(net, X);

  memcpy(predictions[demo_index], prediction, l.outputs * sizeof(float));
  mean_arrays(predictions, demo_frame, l.outputs, avg);
  l.output = last_avg2;
  if (demo_delay == 0)
    l.output = avg;
  if (l.type == DETECTION) {
    get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
  } else if (l.type == REGION) {
    get_region_boxes(l, buff[0].w, buff[0].h, net.w, net.h, demo_thresh, probs, boxes, 0, 0,
                     demo_hier, 1);
  } else {
    error("Last layer must produce detections\n");
  }
  if (nms > 0)
    do_nms_obj(boxes, probs, l.w * l.h * l.n, l.classes, nms);

  if (enable_console_output) {
    printf("\nFPS:%.1f\n", fps);
    printf("Objects:\n\n");
  }
  image display = buff[(buff_index + 2) % 3];
  draw_detections(display, demo_detections, demo_thresh, boxes, probs, demo_names, demo_alphabet,
                  demo_classes);

  // extract the bounding boxes and send them to ROS
  int total = l.w * l.h * l.n;
  int i, j;
  int count = 0;
  for (i = 0; i < total; ++i) {
    float xmin = boxes[i].x - boxes[i].w / 2.;
    float xmax = boxes[i].x + boxes[i].w / 2.;
    float ymin = boxes[i].y - boxes[i].h / 2.;
    float ymax = boxes[i].y + boxes[i].h / 2.;

    if (xmin < 0)
      xmin = 0;
    if (ymin < 0)
      ymin = 0;
    if (xmax > 1)
      xmax = 1;
    if (ymax > 1)
      ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < l.classes; ++j) {
      if (probs[i][j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          ROI_boxes[count].x = x_center;
          ROI_boxes[count].y = y_center;
          ROI_boxes[count].w = BoundingBox_width;
          ROI_boxes[count].h = BoundingBox_height;
          ROI_boxes[count].Class = j;
          ROI_boxes[count].prob = probs[i][j];
          count++;
        }
      }
    }
  }

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) {
    ROI_boxes[0].num = 0;
  } else {
    ROI_boxes[0].num = count;
  }

  demo_index = (demo_index + 1) % demo_frame;
  running = 0;
  return 0;
}

void *YoloObjectDetector::display_in_thread(void *ptr)
{
  show_image_cv(buff[(buff_index + 1) % 3], "Demo", ipl);
  int c = cvWaitKey(wait_key_delay);
  if (c != -1)
    c = c % 256;
  if (c == 10) {
    if (demo_delay == 0)
      demo_delay = 60;
    else if (demo_delay == 5)
      demo_delay = 0;
    else if (demo_delay == 60)
      demo_delay = 5;
    else
      demo_delay = 0;
  } else if (c == 27) {
    demo_done = 1;
    return 0;
  } else if (c == 82) {
    demo_thresh += .02;
  } else if (c == 84) {
    demo_thresh -= .02;
    if (demo_thresh <= .02)
      demo_thresh = .02;
  } else if (c == 83) {
    demo_hier += .02;
  } else if (c == 81) {
    demo_hier -= .02;
    if (demo_hier <= .0)
      demo_hier = .0;
  }
  return 0;
}

void *YoloObjectDetector::display_loop(void *ptr)
{
  while (1) {
    display_in_thread(0);
  }
}

void *YoloObjectDetector::detect_loop(void *ptr)
{
  while (1) {
    detect_in_thread();
  }
}

void YoloObjectDetector::setup_network(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                       char **names, int classes, bool viewimage, int waitkeydelay,
                                       int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                       int frames, int fullscreen, bool enableConsoleOutput)
{
  demo_prefix = prefix;
  demo_delay = delay;
  demo_frame = avg_frames;
  predictions = (float **) calloc(demo_frame, sizeof(float*));
  image **alphabet = load_alphabet_with_file(datafile);
  demo_names = names;
  demo_alphabet = alphabet;
  demo_classes = classes;
  demo_thresh = thresh;
  demo_hier = hier;
  view_image = viewimage;
  wait_key_delay = waitkeydelay;
  full_screen = fullscreen;
  enable_console_output = enableConsoleOutput;
  printf("YOLO_V2\n");
  net = parse_network_cfg(cfgfile);
  if (weightfile) {
    load_weights(&net, weightfile);
  }
  set_batch_network(&net, 1);
}

void YoloObjectDetector::yolo()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!get_image_status()) {
    printf("Waiting for image.\n");
    if (!is_node_running()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  layer l = net.layers[net.n - 1];
  demo_detections = l.n * l.w * l.h;
  int j;

  avg = (float *) calloc(l.outputs, sizeof(float));
  last_avg = (float *) calloc(l.outputs, sizeof(float));
  last_avg2 = (float *) calloc(l.outputs, sizeof(float));
  for (j = 0; j < demo_frame; ++j)
    predictions[j] = (float *) calloc(l.outputs, sizeof(float));

  boxes = (box *) calloc(l.w * l.h * l.n, sizeof(box));
  ROI_boxes = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));
  probs = (float **) calloc(l.w * l.h * l.n, sizeof(float *));
  for (j = 0; j < l.w * l.h * l.n; ++j)
    probs[j] = (float *) calloc(l.classes + 1, sizeof(float));

  IplImage* ROS_img = get_ipl_image();
  buff[0] = ipl_to_image(ROS_img);
  buff[1] = copy_image(buff[0]);
  buff[2] = copy_image(buff[0]);
  buff_letter[0] = letterbox_image(buff[0], net.w, net.h);
  buff_letter[1] = letterbox_image(buff[0], net.w, net.h);
  buff_letter[2] = letterbox_image(buff[0], net.w, net.h);
  ipl = cvCreateImage(cvSize(buff[0].w, buff[0].h), IPL_DEPTH_8U, buff[0].c);

  int count = 0;

  if (!demo_prefix && view_image) {
    cvNamedWindow("Demo", CV_WINDOW_NORMAL);
    if (full_screen) {
      cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
      cvMoveWindow("Demo", 0, 0);
      cvResizeWindow("Demo", 1352, 1013);
    }
  }

  demo_time = get_wall_time();

  while (!demo_done) {
    buff_index = (buff_index + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetch_in_thread, this);
    detect_thread = std::thread(&YoloObjectDetector::detect_in_thread, this);
    if (!demo_prefix) {
      if (count % (demo_delay + 1) == 0) {
        fps = 1. / (get_wall_time() - demo_time);
        demo_time = get_wall_time();
        float *swap = last_avg;
        last_avg = last_avg2;
        last_avg2 = swap;
        memcpy(last_avg, avg, l.outputs * sizeof(float));
      }
      if (view_image) {
        display_in_thread(0);
      }
      publish_in_thread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demo_prefix, count);
      save_image(buff[(buff_index + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!is_node_running()) {
      demo_done = true;
    }
  }

}

IplImage* YoloObjectDetector::get_ipl_image()
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
  IplImage* ROS_img = new IplImage(camImageCopy_);
  return ROS_img;
}

bool YoloObjectDetector::get_image_status(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::is_node_running(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void *YoloObjectDetector::publish_in_thread()
{
  // Publish image.
  cv::Mat cvImage = cv::cvarrToMat(ipl);
  if (!publishDetectionImage(cv::Mat(cvImage))) {
    ROS_DEBUG("Detection image has not been broadcasted.");
  }

  // Publish bounding boxes and detection result.
  int num = ROI_boxes[0].num;
  if (num > 0 && num <= 100) {
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (ROI_boxes[i].Class == j) {
          rosBoxes_[j].push_back(ROI_boxes[i]);
          rosBoxCounter_[j]++;
        }
      }
    }

    std_msgs::Int8 msg;
    msg.data = num;
    objectPublisher_.publish(msg);

    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) {
        darknet_ros_msgs::BoundingBox boundingBox;

        for (int j = 0; j < rosBoxCounter_[i]; j++) {
          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

          boundingBox.Class = classLabels_[i];
          boundingBox.probability = rosBoxes_[i][j].prob;
          boundingBox.xmin = xmin;
          boundingBox.ymin = ymin;
          boundingBox.xmax = xmax;
          boundingBox.ymax = ymax;
          boundingBoxesResults_.boundingBoxes.push_back(boundingBox);
        }
      }
    }
    boundingBoxesResults_.header.stamp = ros::Time::now();
    boundingBoxesResults_.header.frame_id = "detection";
    boundingBoxesPublisher_.publish(boundingBoxesResults_);
  } else {
    std_msgs::Int8 msg;
    msg.data = 0;
    objectPublisher_.publish(msg);
  }
  boundingBoxesResults_.boundingBoxes.clear();
  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  return 0;
}


} /* namespace darknet_ros*/
