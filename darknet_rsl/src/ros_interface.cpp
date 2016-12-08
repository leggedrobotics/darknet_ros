#include "ros_interface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

extern "C" void demo_yolo();
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

cv::Mat cam_image_copy;
char *cfg = "/home/markob/any_ws/src/darknet_rsl/darknet_rsl/cfg/tiny-yolo.cfg";
char *weights = "/home/markob/any_ws/src/darknet_rsl/darknet_rsl/weights/tiny-yolo.weights";
float thresh = 0.2;
const std::string CAMERA_TOPIC_NAME = "/camera/image_raw";

IplImage* get_ipl_image(void)
{
   IplImage* ROS_img = new IplImage(cam_image_copy);
   return ROS_img;
}

class RosInterface
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;

public:
   RosInterface() : _it(_nh)
   {
      _image_sub = _it.subscribe(CAMERA_TOPIC_NAME, 1, &RosInterface::cameraCallback, this);
   }

private:
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
         demo_yolo();
      }
      return;
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ros_interface");

   load_network(cfg, weights, thresh);

   RosInterface ri;
   ros::spin();
   return 0;
}
