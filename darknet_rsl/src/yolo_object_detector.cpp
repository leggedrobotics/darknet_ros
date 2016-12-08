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
extern "C" ROS_box *demo_yolo();
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

// define demo_yolo inputs
char *cfg = "/home/markob/any_ws/src/darknet_rsl/darknet_rsl/cfg/tiny-yolo.cfg";
char *weights = "/home/markob/any_ws/src/darknet_rsl/darknet_rsl/weights/tiny-yolo.weights";
float thresh = 0.2;

const std::string class_labels[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
		     	             "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
		                     "potted plant", "sheep", "sofa", "train", "tv monitor" };
const int num_classes = sizeof(class_labels)/sizeof(class_labels[0]);

cv::Mat cam_image_copy;

// define parameters
const std::string CAMERA_TOPIC_NAME = "/camera/image_raw";
//const std::string CAMERA_WIDTH_PARAM = "/camera/image_width";
//const std::string CAMERA_HEIGHT_PARAM = "/camera/image_height";
const std::string OPENCV_WINDOW = "YOLO object detection";
int FRAME_W;
int FRAME_H;
int FRAME_AREA;
int FRAME_COUNT = 0;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage
IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(cam_image_copy);
   return ROS_img;
}

class yoloObjectDetector
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;
   ros::Publisher _found_object_pub;
   ros::Publisher _bboxes_pub;
   std::vector< std::vector<ROS_box> > _class_bboxes;
   std::vector<int> _class_obj_count;
   std::vector<cv::Scalar> _bbox_colors;
   darknet_rsl::bbox_array _bbox_results_msg;
   ROS_box* _boxes;

public:
   yoloObjectDetector() : _it(_nh), _class_bboxes(num_classes), _class_obj_count(num_classes, 0), _bbox_colors(num_classes)
   {
      int incr = floor(255/num_classes);
      for (int i = 0; i < num_classes; i++)
      {
         _bbox_colors[i] = cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i);
      }

      _image_sub = _it.subscribe(CAMERA_TOPIC_NAME, 1,
	                       &yoloObjectDetector::cameraCallback,this);
      _found_object_pub = _nh.advertise<std_msgs::Int8>("found_object", 1);
      _bboxes_pub = _nh.advertise<darknet_rsl::bbox_array>("YOLO_bboxes", 1);

      cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
   }

   ~yoloObjectDetector()
   {
      cv::destroyWindow(OPENCV_WINDOW);
   }

private:
   void drawBBoxes(cv::Mat &input_frame, std::vector<ROS_box> &class_boxes, int &class_obj_count,
		   cv::Scalar &bbox_color, const std::string &class_label)
   {
      darknet_rsl::bbox bbox_result;

      for (int i = 0; i < class_obj_count; i++)
      {
         int xmin = (class_boxes[i].x - class_boxes[i].w/2)*FRAME_W;
         int ymin = (class_boxes[i].y - class_boxes[i].h/2)*FRAME_H;
         int xmax = (class_boxes[i].x + class_boxes[i].w/2)*FRAME_W;
         int ymax = (class_boxes[i].y + class_boxes[i].h/2)*FRAME_H;

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
          for (int j = 0; j < num_classes; j++)
          {
             if (_boxes[i].Class == j)
             {
                _class_bboxes[j].push_back(_boxes[i]);
                _class_obj_count[j]++;
                std::cout << class_labels[_boxes[i].Class] << " (" << _boxes->prob << ")" << std::endl;
             }
          }
       }

       // send message that an object has been detected
       std_msgs::Int8 msg;
       msg.data = 1;
       _found_object_pub.publish(msg);

       for (int i = 0; i < num_classes; i++)
       {
         if (_class_obj_count[i] > 0) drawBBoxes(input_frame, _class_bboxes[i],
                                                 _class_obj_count[i], _bbox_colors[i], class_labels[i]);
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

      for (int i = 0; i < num_classes; i++)
      {
         _class_bboxes[i].clear();
         _class_obj_count[i] = 0;
      }

      cv::imshow(OPENCV_WINDOW, input_frame);
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
         cam_image_copy = cam_image->image.clone();

         if (FRAME_COUNT == 0)
         {
            runYOLO(cam_image->image);
            FRAME_W = cam_image->image.size().width;
            FRAME_H = cam_image->image.size().height;
         }
         //FRAME_COUNT++;
         if (FRAME_COUNT == 1) FRAME_COUNT = 0;
      }
      return;
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ros_interface");

   //ros::param::get(CAMERA_WIDTH_PARAM, FRAME_W);
   //ros::param::get(CAMERA_HEIGHT_PARAM, FRAME_H);

   load_network(cfg, weights, thresh);

   yoloObjectDetector yod;
   ros::spin();
   return 0;
}
