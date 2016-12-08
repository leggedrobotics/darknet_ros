#include "ros_interface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_rsl/bbox_array.h>
#include <darknet_rsl/bbox.h>
#include <string>

extern "C" {
  #include "box.h"
}

// initialize YOLO functions that are called in this script
extern "C" RosBox_ *demo_yolo();
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

const std::string classLabels_[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
		     	             "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
		                     "potted plant", "sheep", "sofa", "train", "tv monitor" };
const int numClasses_ = sizeof(classLabels_)/sizeof(classLabels_[0]);

cv::Mat camImageCopy_;

// define parameters
std::string cameraTopicName_;
const std::string opencvWindow_ = "YOLO object detection";
int frameWidth_;
int frameHeight_;
int frameArea_;
int frameCount_ = 0;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage
IplImage* get_ipl_image()
{
   IplImage* ROS_img = new IplImage(camImageCopy_);
   return ROS_img;
}

class YoloObjectDetector
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;
   ros::Publisher _found_object_pub;
   ros::Publisher _bboxes_pub;
   std::vector< std::vector<RosBox_> > _class_bboxes;
   std::vector<int> _class_obj_count;
   std::vector<cv::Scalar> _bbox_colors;
   darknet_rsl::bbox_array _bbox_results_msg;
   RosBox_* _boxes;

public:
   YoloObjectDetector() : _it(_nh), _class_bboxes(numClasses_), _class_obj_count(numClasses_, 0), _bbox_colors(numClasses_)
   {
      int incr = floor(255/numClasses_);
      for (int i = 0; i < numClasses_; i++)
      {
         _bbox_colors[i] = cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i);
      }

      _image_sub = _it.subscribe(cameraTopicName_, 1,
	                       &YoloObjectDetector::cameraCallback,this);
      _found_object_pub = _nh.advertise<std_msgs::Int8>("found_object", 1);
      _bboxes_pub = _nh.advertise<darknet_rsl::bbox_array>("YOLO_bboxes", 1);

      cv::namedWindow(opencvWindow_, cv::WINDOW_NORMAL);
   }

   ~YoloObjectDetector()
   {
      cv::destroyWindow(opencvWindow_);
   }

private:
   void drawBBoxes(cv::Mat &input_frame, std::vector<RosBox_> &class_boxes, int &class_obj_count,
		   cv::Scalar &bbox_color, const std::string &class_label)
   {
      darknet_rsl::bbox bbox_result;

      for (int i = 0; i < class_obj_count; i++)
      {
         int xmin = (class_boxes[i].x - class_boxes[i].w/2)*frameWidth_;
         int ymin = (class_boxes[i].y - class_boxes[i].h/2)*frameHeight_;
         int xmax = (class_boxes[i].x + class_boxes[i].w/2)*frameWidth_;
         int ymax = (class_boxes[i].y + class_boxes[i].h/2)*frameHeight_;

         bbox_result.Class = class_label;
         bbox_result.probability = class_boxes[i].prob;
         bbox_result.xmin = xmin;
         bbox_result.ymin = ymin;
         bbox_result.xmax = xmax;
         bbox_result.ymax = ymax;
         _bbox_results_msg.bboxes.push_back(bbox_result);

         // draw bounding box of first object found
         cv::Point topLeftCorner = cv::Point(xmin, ymin);
         cv::Point botRightCorner = cv::Point(xmax, ymax);
         cv::rectangle(input_frame, topLeftCorner, botRightCorner, bbox_color, 2);
         std::ostringstream probability;
         probability << class_boxes[i].prob;
         cv::putText(input_frame, class_label + " (" + probability.str() + ")", cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
                     1.0, bbox_color, 2.0);
      }
   }

   void runYOLO(cv::Mat &full_frame)
   {
      cv::Mat input_frame = full_frame.clone();

      // run yolo and get bounding boxes for objects
      _boxes = demo_yolo();

      // get the number of bounding boxes found
      int num = _boxes[0].num;

      // if at least one bbox found, draw box
      if (num > 0  && num <= 100)
      {
        std::cout << "# Objects: " << num << std::endl;

        // split bounding boxes by class
       for (int i = 0; i < num; i++)
       {
          for (int j = 0; j < numClasses_; j++)
          {
             if (_boxes[i].Class == j)
             {
                _class_bboxes[j].push_back(_boxes[i]);
                _class_obj_count[j]++;
                std::cout << classLabels_[_boxes[i].Class] << " (" << _boxes->prob << ")" << std::endl;
             }
          }
       }

       // send message that an object has been detected
       std_msgs::Int8 msg;
       msg.data = 1;
       _found_object_pub.publish(msg);

       for (int i = 0; i < numClasses_; i++)
       {
         if (_class_obj_count[i] > 0) drawBBoxes(input_frame, _class_bboxes[i],
                                                 _class_obj_count[i], _bbox_colors[i], classLabels_[i]);
       }
       _bboxes_pub.publish(_bbox_results_msg);
       _bbox_results_msg.bboxes.clear();
      }
      else
      {
        std_msgs::Int8 msg;
        msg.data = 0;
        _found_object_pub.publish(msg);
      }

      for (int i = 0; i < numClasses_; i++)
      {
         _class_bboxes[i].clear();
         _class_obj_count[i] = 0;
      }

      cv::imshow(opencvWindow_, input_frame);
      cv::waitKey(3);
   }

   void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      std::cout << "usb image received" << std::endl;

      cv_bridge::CvImagePtr cam_image;

      try
      {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      if (cam_image)
      {
         camImageCopy_ = cam_image->image.clone();

         if (frameCount_ == 0)
         {
            runYOLO(cam_image->image);
            frameWidth_ = cam_image->image.size().width;
            frameHeight_ = cam_image->image.size().height;
         }
         //frameCount_++;
         if (frameCount_ == 1) frameCount_ = 0;
      }
      return;
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ros_interface");

   std::string param;
   float thresh;
   ros::param::get("/darknet_rsl/object_threshold", thresh);
   ros::param::get("/darknet_rsl/camera_topic_name", cameraTopicName_);
   ros::param::get("/darknet_rsl/weights_path", param);
   char *weights = new char[param.length() + 1];
   strcpy(weights, param.c_str());
   ros::param::get("/darknet_rsl/cfg_path", param);
   char *cfg = new char[param.length() + 1];
   strcpy(cfg, param.c_str());

   load_network(cfg, weights, thresh);

   YoloObjectDetector yod;
   ros::spin();
   return 0;
}
