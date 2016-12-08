#ifndef ros_interface
#define ros_interface

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/gpu/gpu.hpp>

IplImage* get_Ipl_image(void);

typedef struct {
  float x, y, w, h;
  int num, Class;
} ROS_box;

#endif
