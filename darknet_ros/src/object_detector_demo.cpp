/*
 * object_detector_demo.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

#include "darknet_ros/YoloObjectDetector.h"

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

#define FRAMES 1

#ifdef OPENCV

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

extern "C" image ipl_to_image(IplImage* src);
extern "C" void convert_yolo_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);
extern "C" void draw_yolo(image im, int num, float thresh, box *boxes, float **probs);

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static float **probs;
static box *boxes;
static network net;
static image in   ;
static image in_s ;
static image det  ;
static image det_s;
static image disp = {0};
static float fps = 0;
static float demo_thresh = 0;
static float demo_hier = .5;
static int delay;
static int count;

static float *predictions[FRAMES];
static int demo_index = 0;
static image images[FRAMES];
static float *avg;

static darknet_ros::RosBox_ *ROI_boxes;
static bool view_image;
static bool enable_console_output;
static int wait_key_delay;
static int full_screen;

static pthread_t fetch_thread;
static pthread_t detect_thread;
static double tval_before, tval_after;

void *fetch_in_thread(void *ptr) {
  IplImage* ROS_img = darknet_ros::get_ipl_image();
  in = ipl_to_image(ROS_img);
  delete ROS_img;
  ROS_img = NULL;
  if(!in.data) {
    error("Stream closed.");
  }
  in_s = letterbox_image(in, net.w, net.h);
  return 0;
}

void *detect_in_thread(void *ptr) {
  float nms = .4;

  layer l = net.layers[net.n-1];
  float *X = det_s.data;
  float *prediction = network_predict(net, X);

  memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
  mean_arrays(predictions, FRAMES, l.outputs, avg);
  l.output = avg;

  free_image(det_s);
  if(l.type == DETECTION) {
    get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
  } else if (l.type == REGION){
    get_region_boxes(l, in.w, in.h, net.w, net.h, demo_thresh, probs, boxes, 0, 0, demo_hier, 1);
  } else {
    error("Last layer must produce detections\n");
  }
  if (nms > 0) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
  //printf("\033[2J");
  //printf("\033[1;1H");
  if (enable_console_output) {
    printf("\nFPS:%.1f\n",fps);
  }

  if(view_image)
  {
    if (enable_console_output) {
      printf("Objects:\n\n");
    }
    images[demo_index] = det;
    det = images[(demo_index + FRAMES/2 + 1)%FRAMES];
    demo_index = (demo_index + 1)%FRAMES;
    draw_detections(det, l.w*l.h*l.n, demo_thresh, boxes, probs, demo_names, demo_alphabet, demo_classes);
  }

  // extract the bounding boxes and send them to ROS
  int total = l.w*l.h*l.n;
  int i, j;
  int count = 0;
  for(i = 0; i < total; ++i)
  {
    float xmin = boxes[i].x - boxes[i].w/2.;
    float xmax = boxes[i].x + boxes[i].w/2.;
    float ymin = boxes[i].y - boxes[i].h/2.;
    float ymax = boxes[i].y + boxes[i].h/2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for(j = 0; j < l.classes; ++j)
    {
      if (probs[i][j])
      {
        float x_center = (xmin+xmax)/2;
        float y_center = (ymin+ymax)/2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01)
        {
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
  if (count == 0)
  {
    ROI_boxes[0].num = 0;
  }
  else
  {
    ROI_boxes[0].num = count;
  }

  return 0;
}

double get_wall_time() {
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

extern "C" void load_network_demo(char *cfgfile, char *weightfile, char *datafile,
                                  float thresh,
                                  char **names, int classes,
                                  bool viewimage, int waitkeydelay,
                                  int frame_skip,
                                  float hier,
                                  int w, int h, int frames, int fullscreen,
                                  bool enableConsoleOutput) {
  image **alphabet = load_alphabet_with_file(datafile);
  delay = frame_skip;
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
  if(weightfile) {
    load_weights(&net, weightfile);
  }
  set_batch_network(&net, 1);

  srand(2222222);

  layer l = net.layers[net.n-1];
  int j;

  avg = (float *) calloc(l.outputs, sizeof(float));
  for(j = 0; j < FRAMES; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));
  for(j = 0; j < FRAMES; ++j) images[j] = make_image(1,1,3);

  boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
  ROI_boxes = (darknet_ros::RosBox_ *)calloc(l.side*l.side*l.n, sizeof(darknet_ros::RosBox_));
  probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
  for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes, sizeof(float *));

  count = 0;

  tval_before = get_wall_time();
}

extern "C" darknet_ros::RosBox_ *demo_yolo() {
  if (count == 0) {
    ++count;
    fetch_in_thread(0);
    det = in;
    det_s = in_s;

    fetch_in_thread(0);
    detect_in_thread(0);
    disp = det;
    det = in;
    det_s = in_s;

    int j;
    for(j = 0; j < FRAMES/2; ++j){
      fetch_in_thread(0);
      detect_in_thread(0);
      disp = det;
      det = in;
      det_s = in_s;
    }

    if(view_image)
    {
      cvNamedWindow("YOLO_V2", CV_WINDOW_NORMAL);
      if(full_screen){
        cvSetWindowProperty("YOLO_V2", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
      } else {
        cvMoveWindow("YOLO_V2", 0, 0);
        cvResizeWindow("YOLO_V2", 1352, 1013);
      }
    }
  } else {
    ++count;
    fetch_in_thread(0);
    det = in;
    det_s = in_s;

    if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
    if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");\
    if(view_image)
    {
      show_image(disp, "YOLO_V2");
      cvWaitKey(wait_key_delay);
    }
    pthread_join(fetch_thread, 0);
    pthread_join(detect_thread, 0);

    free_image(disp);
    disp  = det;
    free_image(in);
    free_image(in_s);

    tval_after = get_wall_time();
    float curr = 1./(tval_after - tval_before);
    fps = curr;
    tval_before = tval_after;
  }

  return ROI_boxes;
}
#else
extern "C"  void load_network_demo(char *cfgfile, char *weightfile, char *datafile,
                                   float thresh,
                                   char **names, int classes,
                                   bool viewimage, int waitkeydelay,
                                   int frame_skip,
                                   float hier,
                                   int w, int h, int frames, int fullscreen,
                                   bool enableConsoleOutput)
{
  fprintf(stderr, "YOLO demo needs OpenCV for webcam images.\n");
}

extern "C"  darknet_ros::RosBox_ *demo_yolo()
{
  fprintf(stderr, "YOLO demo needs OpenCV for webcam images.\n");
}
#endif
