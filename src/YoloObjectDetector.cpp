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
// #include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

YoloObjectDetector::YoloObjectDetector()
    : Node("darknet_ros"),
      numClasses_(0),
      classLabels_(0)
{
  RCLCPP_INFO(get_logger(), "[YoloObjectDetector] Node started.");

  declare_parameter("network.detection_classes", std::vector<std::string>(0));

  declare_parameter("network.threshold", 0.3f);
  declare_parameter("network.weights_file", std::string("yolov4-tiny.weights"));
  declare_parameter("weights_path", std::string("/default"));

  declare_parameter("network.config_file", std::string("yolov4-tiny.cfg"));
  declare_parameter("config_path", std::string("/default"));

}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    std::unique_lock<std::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Set vector sizes.
  get_parameter("network.detection_classes", classLabels_);
  numClasses_ = classLabels_.size();

  return true;
}

void YoloObjectDetector::init()
{
  // Read parameters from config file.
  if (!readParameters()) {
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  get_parameter("network.threshold", thresh);
  
  // Path to weights file.
  get_parameter("network.weights_file", weightsModel);
  get_parameter("weights_path", weightsPath);
  weightsPath += "/" + weightsModel;

  char *weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  get_parameter("network.config_file", configModel);
  get_parameter("config_path", configPath);
  configPath += "/" + configModel;
  char *cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  char *data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  char **detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_,
                0, 0, 1, 0.5, 0, 0, 0);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  // Ouput topic ~/detections [vision_msgs/msg/Detection2DArray]
  detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "~/detections", 1);

  // Configure RTSP Streamer
  rtsp_streamer_.on_configure_writer(1920, 1080);
  rtsp_streamer_.on_configure_reader(std::bind(&YoloObjectDetector::on_image_callback, this, std::placeholders::_1), "rtsp://127.0.0.1:8554/drone");
}

void YoloObjectDetector::on_image_callback(const cv::Mat& image)
{
  if (!image.empty()) {
    {
      std::unique_lock<std::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = image.clone();
    }
    {
      std::unique_lock<std::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = image.cols;
    frameHeight_ = image.rows;
  }
}

int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  // int i;
  // int count = 0;
  // for(i = 0; i < net->n; ++i){
  //   layer l = net->layers[i];
  //   if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
  //     memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
  //     count += l.outputs;
  //   }
  // }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  // int i, j;
  // int count = 0;
  // fill_cpu(demoTotal_, 0, avg_, 1);
  // for(j = 0; j < demoFrame_; ++j){
  //   axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  // }
  // for(i = 0; i < net->n; ++i){
  //   layer l = net->layers[i];
  //   if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
  //     memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
  //     count += l.outputs;
  //   }
  // }
  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes, 1); // letter box
  return dets;
}

void *YoloObjectDetector::detectInThread()
{
  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float *prediction = network_predict(*net_, X);

  // rememberNetwork(net_);
  detection *dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  image display = buff_[(buffIndex_+2) % 3];
  
  draw_detections_v3(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, false);
  // extract the bounding boxes and send them to ROS
  vision_msgs::msg::Detection2DArray detections_array_msg;
  detections_array_msg.detections.reserve(nboxes);
  for (size_t i = 0; i < nboxes; ++i) 
  {
    detections_array_msg.detections.emplace_back();
    auto & detection_msg = detections_array_msg.detections.back();
    detection_msg.bbox.center.x = dets[i].bbox.x;
    detection_msg.bbox.center.y = dets[i].bbox.y;
    detection_msg.bbox.size_x = dets[i].bbox.w;
    detection_msg.bbox.size_y = dets[i].bbox.h;

    float max_score = 0.0f;
    std::string max_class;
    for (int cls = 0; cls < dets[i].classes; ++cls) {
      if (dets[i].prob[cls] > max_score) {
        max_score = dets[i].prob[cls];
        max_class = classLabels_[cls];
      }
    }
    detection_msg.results.emplace_back();
    auto & hypothesis = detection_msg.results.back();
    hypothesis.id = max_class;
    hypothesis.score = max_score;
  }
  detections_array_msg.header.stamp = rclcpp::Time(0);
  detections_pub_->publish(detections_array_msg);
  
  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

void* YoloObjectDetector::fetchInThread() {
  {
    free_image(buff_[buffIndex_]);
    std::shared_lock<std::shared_mutex> lock(mutexImageCallback_);
    buff_[buffIndex_] = mat_to_image(camImageCopy_);
  }
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  return 0;
}


float get_pixel_cp(image m, int x, int y, int c)
{
    assert(x < m.w && y < m.h && c < m.c);
    return m.data[c*m.h*m.w + y*m.w + x];
}

image **load_alphabet_with_file_cp(char *datafile) {
  int i, j;
  const int nsize = 8;
  image **alphabets = (image**)calloc(nsize, sizeof(image));
  char* labels = "/labels/%d_%d.png";
  char * files = (char *) malloc(1 + strlen(datafile)+ strlen(labels) );
  strcpy(files, datafile);
  strcat(files, labels);
  for(j = 0; j < nsize; ++j){
    alphabets[j] = (image*)calloc(128, sizeof(image));
    for(i = 32; i < 127; ++i){
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0);
    }
  }
  return alphabets;
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                      int frames)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file_cp(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo()
{
  static int start_count = 0;
  const auto wait_duration = std::chrono::milliseconds(2000);
  printf("Waiting for image.\n");
  while (!getImageStatus()) {
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  // int i;
  // demoTotal_ = sizeNetwork(net_);
  // predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
  // for (i = 0; i < demoFrame_; ++i){
  //     predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
  // }
  // avg_ = (float *) calloc(demoTotal_, sizeof(float));

  // layer l = net_->layers[net_->n - 1];

  {
    std::shared_lock<std::shared_mutex> lock(mutexImageCallback_);
    buff_[0] = mat_to_image(camImageCopy_);
  }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  disp_ = image_to_mat(buff_[0]);

  int count = 0;

  std::thread detect_thread;
  std::thread fetch_thread;
  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    
    generate_image_cp(buff_[(buffIndex_ + 1)%3], disp_);
    rtsp_streamer_.publish_rtsp_stream(disp_);
      
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }
}

bool YoloObjectDetector::getImageStatus(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void YoloObjectDetector::generate_image_cp(image p, cv::Mat& disp) {
  int x, y, k;
  if (p.c == 3) rgbgr_image(p);
  // normalize_image(copy);

  int step = disp.step;
  for (y = 0; y < p.h; ++y) {
    for (x = 0; x < p.w; ++x) {
      for (k = 0; k < p.c; ++k) {
        disp.data[y * step + x * p.c + k] = (unsigned char)(get_pixel_cp(p, x, y, k) * 255);
      }
    }
  }
}

} /* namespace darknet_ros*/