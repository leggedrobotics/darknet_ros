#include "ros_interface.h"

extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include <sys/time.h>
}

#define FRAMES 1

#ifdef OPENCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
extern "C" image ipl_to_image(IplImage* src);
extern "C" void convert_yolo_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);
extern "C" void draw_yolo(image im, int num, float thresh, box *boxes, float **probs);

extern "C" char *voc_names;
static image **demo_alphabet;
extern "C" int demo_classes;

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

static float *predictions[FRAMES];
static int demo_index = 0;
static image images[FRAMES];
static float *avg;

void *fetch_in_thread(void *ptr)
{
   IplImage* ROS_img = get_ipl_image();
   in = ipl_to_image(ROS_img);
   delete ROS_img;
   ROS_img = NULL;
   rgbgr_image(in);
   in_s = resize_image(in, net.w, net.h);
   return 0;
}

void *detect_in_thread(void *ptr)
{
	float nms = .4;

	layer l = net.layers[net.n-1];
	float *X = det_s.data;
	float *prediction = network_predict(net, X);

	memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
	mean_arrays(predictions, FRAMES, l.outputs, avg);
	l.output = avg;

	free_image(det_s);
	if(l.type == DETECTION){
		get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
	} else if (l.type == REGION){
		get_region_boxes(l, 1, 1, demo_thresh, probs, boxes, 0, 0);
	} else {
		error("Last layer must produce detections\n");
	}
	if (nms > 0) do_nms(boxes, probs, l.w*l.h*l.n, l.classes, nms);
	printf("\033[2J");
	printf("\033[1;1H");
	printf("\nFPS:%.1f\n",fps);
	printf("Objects:\n\n");

	images[demo_index] = det;
	det = images[(demo_index + FRAMES/2 + 1)%FRAMES];
	demo_index = (demo_index + 1)%FRAMES;

	draw_detections(det, l.w*l.h*l.n, demo_thresh, boxes, probs, &voc_names, demo_alphabet, demo_classes);

	return 0;
}

double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

extern "C" void load_network(char *cfgfile, char *weightfile, char *datafile, float thresh)
{
	image **alphabet = load_alphabet_with_file(datafile);
	demo_alphabet = alphabet;
	demo_thresh = thresh;
	printf("Demo\n");
	net = parse_network_cfg(cfgfile);
	if(weightfile){
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
	probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
	for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes, sizeof(float *));

	cvNamedWindow("YOLO V2", CV_WINDOW_NORMAL);
	cvMoveWindow("YOLO V2", 0, 0);
	cvResizeWindow("YOLO V2", 1352, 1013);
}

extern "C" void demo_yolo()
{
	int j;
	char *prefix = 0;
	int frame_skip = 20;
	int delay = frame_skip;
	pthread_t fetch_thread;
	pthread_t detect_thread;

	fetch_in_thread(0);
	det = in;
	det_s = in_s;

	fetch_in_thread(0);
	detect_in_thread(0);
	disp = det;
	det = in;
	det_s = in_s;

	for(j = 0; j < FRAMES/2; ++j){
		fetch_in_thread(0);
		detect_in_thread(0);
		disp = det;
		det = in;
		det_s = in_s;
	}

	struct timeval tval_before, tval_after, tval_result;
	gettimeofday(&tval_before, NULL);
	if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
	if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
	show_image(disp, "YOLO V2");
	free_image(disp);
	cvWaitKey(1);
	pthread_join(fetch_thread, 0);
	pthread_join(detect_thread, 0);

	disp  = det;
	det   = in;
	det_s = in_s;
	free_image(in);
	free_image(in_s);
	free_image(disp);

	gettimeofday(&tval_after, NULL);
	timersub(&tval_after, &tval_before, &tval_result);
	float curr = 1000000.f/((long int)tval_result.tv_usec);
	fps = .9*fps + .1*curr;

}
#else
extern "C" void demo_yolo(char *cfgfile, char *weightfile, float thresh, int cam_index){
    fprintf(stderr, "YOLO demo needs OpenCV for webcam images.\n");
}
#endif
