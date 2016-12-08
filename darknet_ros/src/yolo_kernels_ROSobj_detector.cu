#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

#include "ROS_interface.h"

extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include <sys/time.h>
}

#ifdef OPENCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
extern "C" image ipl_to_image(IplImage* src);
extern "C" void convert_yolo_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);
extern "C" void draw_yolo(image im, int num, float thresh, box *boxes, float **probs);

extern "C" char *voc_names[];
extern "C" image voc_labels[];

static float **probs;
static box *boxes;
static network net;
static image in;
static image in_s;
static image det_s;
static float fps = 0;
static float demo_thresh = 0;

static ROS_box *ROI_boxes;

void fetch_in_thread()
{
   IplImage* ROS_img = get_Ipl_image();
   in = ipl_to_image(ROS_img);
   delete ROS_img;
   ROS_img = NULL;
   rgbgr_image(in);
   in_s = resize_image(in, net.w, net.h);
   free_image(in);
   return;
}

void detect_in_thread()
{
    float nms = .4;

    detection_layer l = net.layers[net.n-1];
    float *X = det_s.data;
    float *predictions = network_predict(net, X);
    free_image(det_s);
    convert_yolo_detections(predictions, l.classes, l.n, l.sqrt, l.side, 1, 1, demo_thresh, probs, boxes, 0);
    if (nms > 0) do_nms(boxes, probs, l.side*l.side*l.n, l.classes, nms);
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.0f\n",fps);

    // extract the bounding boxes and send them to ROS
    int total = l.side*l.side*l.n;
    int i, j;
    int count = 0;
    for(i = 0; i < total; ++i){
        float xmin = boxes[i].x - boxes[i].w/2.;
        float xmax = boxes[i].x + boxes[i].w/2.;
        float ymin = boxes[i].y - boxes[i].h/2.;
        float ymax = boxes[i].y + boxes[i].h/2.;

        if (xmin < 0) xmin = 0;
        if (ymin < 0) ymin = 0;
        if (xmax > 1) xmax = 1;
        if (ymax > 1) ymax = 1;

	// iterate through possible boxes and collect the bounding boxes
        for(j = 0; j < l.classes; ++j){
            if (probs[i][j]) {
		float x_center = (xmin+xmax)/2;
		float y_center = (ymin+ymax)/2;
		float bbox_width = xmax - xmin;
		float bbox_height = ymax - ymin;

		// define bounding box 
		// bbox must be 1% size of frame (3.2x2.4 pixels)
		if (bbox_width > 0.01 && bbox_height > 0.01) {
   		     ROI_boxes[count].x = x_center;
                     ROI_boxes[count].y = y_center;
                     ROI_boxes[count].w = bbox_width;
                     ROI_boxes[count].h = bbox_height;
                     ROI_boxes[count].Class = j;
		     count++;
		}
		
		//printf("%f %f\n", x_center*320, y_center*240);
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

    return;
}

extern "C" void load_network(char *cfgfile, char *weightfile, float thresh, int cam_index)
{
    demo_thresh = thresh;
    printf("YOLO demo\n");
    net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);

    srand(2222222);

    detection_layer l = net.layers[net.n-1];
    int j;

    boxes = (box *)calloc(l.side*l.side*l.n, sizeof(box));
    ROI_boxes = (ROS_box *)calloc(l.side*l.side*l.n, sizeof(ROS_box));
    probs = (float **)calloc(l.side*l.side*l.n, sizeof(float *));
    for(j = 0; j < l.side*l.side*l.n; ++j) probs[j] = (float *)calloc(l.classes, sizeof(float *));
}

extern "C" ROS_box *demo_yolo()
{
    struct timeval tval_before, tval_after, tval_result;
    gettimeofday(&tval_before, NULL);
    fetch_in_thread();
    detect_in_thread();
    det_s = in_s;

    gettimeofday(&tval_after, NULL);
    timersub(&tval_after, &tval_before, &tval_result);
    float curr = 1000000.f/((long int)tval_result.tv_usec);
    fps = .9*fps + .1*curr;
    return ROI_boxes;
}
#else
extern "C" void demo_yolo(char *cfgfile, char *weightfile, float thresh, int cam_index){
    fprintf(stderr, "YOLO demo needs OpenCV for webcam images.\n");
}
#endif

