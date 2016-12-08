//
//  MultiClassObjectDetector.cpp
//  pr2_perception
//
//  Created by Xun Wang on 12/05/16.
//  Copyright (c) 2016 Xun Wang. All rights reserved.
//

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/bind.hpp>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <time.h>

#include <darknet/image.h>
#include "MultiClassObjectDetector.h"

#include "dn_object_detect/DetectedObjects.h"

namespace uts_perp {

using namespace std;
using namespace cv;
  
static const int kPublishFreq = 10; // darknet can work reasonably around 5FPS
static const string kDefaultDevice = "/wide_stereo/right/image_rect_color";
static const string kYOLOModel = "data/yolo.weights";
static const string kYOLOConfig = "data/yolo.cfg";

static const char * VoClassNames[] = { "aeroplane", "bicycle", "bird", // should not hard code these name
                              "boat", "bottle", "bus", "car",
                              "cat", "chair", "cow", "diningtable",
                              "dog", "horse", "motorbike",
                              "person", "pottedplant", "sheep",
                              "sofa", "train", "tvmonitor"
                            };

static int NofVoClasses = sizeof( VoClassNames ) / sizeof( VoClassNames[0] );

/*
extern "C" {
void convert_yolo_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);

}
*/

static inline long timediff_usec( timespec start, timespec end )
{
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    return (end.tv_sec - start.tv_sec - 1) * 1E6 + (1E9 + end.tv_nsec - start.tv_nsec) / 1E3;
  }
  else {
    return (end.tv_sec - start.tv_sec) * 1E6 + (end.tv_nsec - start.tv_nsec) / 1E3;
  }
}

MultiClassObjectDetector::MultiClassObjectDetector() :
  imgTrans_( priImgNode_ ),
  initialised_( false ),
  doDetection_( false ),
  debugRequests_( 0 ),
  srvRequests_( 0 ),
  procThread_( NULL )
{
  priImgNode_.setCallbackQueue( &imgQueue_ );
}

MultiClassObjectDetector::~MultiClassObjectDetector()
{
}

void MultiClassObjectDetector::init()
{
  NodeHandle priNh( "~" );
  std::string yoloModelFile;
  std::string yoloConfigFile;
  
  priNh.param<std::string>( "camera", cameraDevice_, kDefaultDevice );
  priNh.param<std::string>( "yolo_model", yoloModelFile, kYOLOModel );
  priNh.param<std::string>( "yolo_config", yoloConfigFile, kYOLOConfig );
  priNh.param( "threshold", threshold_, 0.2f );
  
  const boost::filesystem::path modelFilePath = yoloModelFile;
  const boost::filesystem::path configFilepath = yoloConfigFile;
  
  if (boost::filesystem::exists( modelFilePath ) && boost::filesystem::exists( configFilepath )) {
    darkNet_ = parse_network_cfg( (char*)yoloConfigFile.c_str() );
    load_weights( darkNet_, (char*)yoloModelFile.c_str() );
    detectLayer_ = darkNet_->layers[darkNet_->n-1];
    printf( "detect layer side = %d n = %d\n", detectLayer_.side, detectLayer_.n );
    maxNofBoxes_ = detectLayer_.side * detectLayer_.side * detectLayer_.n;
    set_batch_network( darkNet_, 1 );
    srand(2222222);
  }
  else {
    ROS_ERROR( "Unable to find YOLO darknet configuration or model files." );
    return;
  }

  ROS_INFO( "Loaded detection model data." );
  
  procThread_ = new AsyncSpinner( 1, &imgQueue_ );
  procThread_->start();

  initialised_ = true;

  dtcPub_ = priImgNode_.advertise<dn_object_detect::DetectedObjects>( "/dn_object_detect/detected_objects", 1,
      boost::bind( &MultiClassObjectDetector::startDetection, this ),
      boost::bind( &MultiClassObjectDetector::stopDetection, this) );

  imgPub_ = imgTrans_.advertise( "/dn_object_detect/debug_view", 1,
      boost::bind( &MultiClassObjectDetector::startDebugView, this ),
      boost::bind( &MultiClassObjectDetector::stopDebugView, this) );
}

void MultiClassObjectDetector::fini()
{
  srvRequests_ = 1; // reset requests
  this->stopDetection();

  if (procThread_) {
    delete procThread_;
    procThread_ = NULL;
  }
  free_network( darkNet_ );
  initialised_ = false;
}

void MultiClassObjectDetector::continueProcessing()
{
  ros::spin();
}
  
void MultiClassObjectDetector::doObjectDetection()
{
  //ros::Rate publish_rate( kPublishFreq );
  ros::Time ts;

  float nms = 0.5;

  box * boxes = (box *)calloc( maxNofBoxes_, sizeof( box ) );
  float **probs = (float **)calloc( maxNofBoxes_, sizeof(float *));
  for(int j = 0; j < maxNofBoxes_; ++j) {
    probs[j] = (float *)calloc( detectLayer_.classes, sizeof(float *) );
  }

  timespec time1, time2;
  DetectedList detectObjs;
  detectObjs.reserve( 30 ); // silly hardcode

  long interval = long( 1.0 / double( kPublishFreq ) * 1E6);
  long proctime = 0;

  while (doDetection_) {
    clock_gettime( CLOCK_MONOTONIC, &time1 );
    {
      boost::mutex::scoped_lock lock( mutex_ );
      if (imgMsgPtr_.get() == NULL) {
        imageCon_.wait( lock );
        continue;
      }
      try {
        cv_ptr_ = cv_bridge::toCvCopy( imgMsgPtr_, sensor_msgs::image_encodings::RGB8 );
        ts = imgMsgPtr_->header.stamp;
      }
      catch (cv_bridge::Exception & e) {
        ROS_ERROR( "Unable to convert image message to mat." );
        imgMsgPtr_.reset();
        continue;
      }
      imgMsgPtr_.reset();
    }

    if (cv_ptr_.get()) {
      IplImage img = cv_ptr_->image;
      image im = ipl_to_image( &img );
      image sized = resize_image( im, darkNet_->w, darkNet_->h );
      float *X = sized.data;
      float *predictions = network_predict( darkNet_, X );
      //printf("%s: Predicted in %f seconds.\n", input, sec(clock()-time));
      convert_yolo_detections( predictions, detectLayer_.classes, detectLayer_.n, detectLayer_.sqrt,
          detectLayer_.side, 1, 1, threshold_, probs, boxes, 0);
      if (nms) {
        do_nms_sort( boxes, probs, maxNofBoxes_,
            detectLayer_.classes, nms );
      }

      this->consolidateDetectedObjects( &im, boxes, probs, detectObjs );
      //draw_detections(im, l.side*l.side*l.n, thresh, boxes, probs, voc_names, 0, 20);
      free_image(im);
      free_image(sized);
      this->publishDetectedObjects( detectObjs );
      if (debugRequests_ > 0)
        this->drawDebug( detectObjs );
    }
    cv_ptr_.reset();

    clock_gettime( CLOCK_MONOTONIC, &time2 );
    proctime = timediff_usec( time1, time2 );
    //printf( "detect process time %li usec\n",  proctime);
    if (interval > proctime)
      usleep( interval - proctime );
  }

  // clean up
  for(int j = 0; j < maxNofBoxes_; ++j) {
    free( probs[j] );
  }
  free( probs );
  free( boxes );
}
  
void MultiClassObjectDetector::processingRawImages( const sensor_msgs::ImageConstPtr& msg )
{
  // assume we cannot control the framerate (i.e. default 30FPS)
  boost::mutex::scoped_lock lock( mutex_, boost::try_to_lock );

  if (lock) {
    imgMsgPtr_ = msg;
    imageCon_.notify_one();
  }
}

void MultiClassObjectDetector::startDebugView()
{
  if (debugRequests_ == 0)
    this->startDetection();
  
  debugRequests_++;
}

void MultiClassObjectDetector::stopDebugView()
{
  debugRequests_--;
  if (debugRequests_ <= 0)
    this->stopDetection();

}

void MultiClassObjectDetector::startDetection()
{
  if (!initialised_) {
    ROS_ERROR( "Detector is not initialised correctly!\n" );
    return;
  }
  srvRequests_ ++;
  if (srvRequests_ > 1)
    return;

  doDetection_ = true;
  cv_ptr_.reset();
  imgMsgPtr_.reset();

  image_transport::TransportHints hints( "compressed" );
  imgSub_ = imgTrans_.subscribe( cameraDevice_, 1,
                                  &MultiClassObjectDetector::processingRawImages, this, hints );
  
  object_detect_thread_ = new boost::thread( &MultiClassObjectDetector::doObjectDetection, this );

  ROS_INFO( "Starting multi-class object detection service." );
}

void MultiClassObjectDetector::stopDetection()
{
  srvRequests_--;
  if (srvRequests_ > 0)
    return;

  doDetection_ = false;
 
  if (object_detect_thread_) {
    object_detect_thread_->join();
    delete object_detect_thread_;
    object_detect_thread_ = NULL;
  }

  imgSub_.shutdown();

  ROS_INFO( "Stopping multi-class object detection service." );
}

void MultiClassObjectDetector::publishDetectedObjects( const DetectedList & objs )
{
  dn_object_detect::DetectedObjects tObjMsg;
  tObjMsg.header = cv_ptr_->header;
  
  tObjMsg.objects.resize( objs.size() );

  for (size_t i = 0; i < objs.size(); i++) {
    tObjMsg.objects[i] = objs[i];
  }

  dtcPub_.publish( tObjMsg );
}

void MultiClassObjectDetector::drawDebug( const DetectedList & objs )
{
  cv::Scalar boundColour( 255, 0, 255 );
  cv::Scalar connColour( 209, 47, 27 );

  for (size_t i = 0; i < objs.size(); i++) {
    dn_object_detect::ObjectInfo obj = objs[i];
    cv::rectangle( cv_ptr_->image, cv::Rect(obj.tl_x, obj.tl_y, obj.width, obj.height),
        boundColour, 2 );

    // only write text on the head or body if no head is detected.
    std::string box_text = format( "%s prob=%.2f", obj.type.c_str(), obj.prob );
    // Calculate the position for annotated text (make sure we don't
    // put illegal values in there):
    cv::Point2i txpos( std::max(obj.tl_x - 10, 0),
                      std::max(obj.tl_y - 10, 0) );
    // And now put it into the image:
    putText( cv_ptr_->image, box_text, txpos, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
  }
  imgPub_.publish( cv_ptr_->toImageMsg() );
}

void MultiClassObjectDetector::consolidateDetectedObjects( const image * im, box * boxes,
      float **probs, DetectedList & objList )
{
  //printf( "max_nofb %d, NofVoClasses %d\n", max_nofb, NofVoClasses );
  int objclass = 0;
  float prob = 0.0;

  objList.clear();

  for(int i = 0; i < maxNofBoxes_; ++i){
    objclass = max_index( probs[i], NofVoClasses );
    prob = probs[i][objclass];

    if (prob > threshold_) {
      int width = pow( prob, 0.5 ) * 10 + 1;
      dn_object_detect::ObjectInfo newObj;
      newObj.type = VoClassNames[objclass];
      newObj.prob = prob;

      //printf("%s: %.2f\n", VoClassNames[objclass], prob);
      /*
      int offset = class * 17 % classes;
      float red = get_color(0,offset,classes);
      float green = get_color(1,offset,classes);
      float blue = get_color(2,offset,classes);
      float rgb[3];
      rgb[0] = red;
      rgb[1] = green;
      rgb[2] = blue;
      box b = boxes[i];
      */

      int left  = (boxes[i].x - boxes[i].w/2.) * im->w;
      int right = (boxes[i].x + boxes[i].w/2.) * im->w;
      int top   = (boxes[i].y - boxes[i].h/2.) * im->h;
      int bot   = (boxes[i].y + boxes[i].h/2.) * im->h;

      if (right > im->w-1)  right = im->w-1;
      if (bot > im->h-1)    bot = im->h-1;

      newObj.tl_x = left < 0 ? 0 : left;
      newObj.tl_y = top < 0 ? 0 : top;
      newObj.width = right - newObj.tl_x;
      newObj.height = bot - newObj.tl_y;
      objList.push_back( newObj );
      //draw_box_width(im, left, top, right, bot, width, red, green, blue);
      //if (labels) draw_label(im, top + width, left, labels[class], rgb);
    }
  }
}

} // namespace uts_perp
