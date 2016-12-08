#ifndef YOLO_H
#define YOLO_H

#include "network.h"
#include "detection_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#endif

#if defined (__cplusplus)
extern "C" {
#endif

void train_yolo(char *cfgfile, char *weightfile);
void convert_yolo_detections(float *predictions, int classes, int num,
    int square, int side, int w, int h, float thresh, float **probs,
    box *boxes, int only_objectness);
void validate_yolo(char *cfgfile, char *weightfile);
void validate_yolo_recall(char *cfgfile, char *weightfile);

#if defined (__cplusplus)
}
#endif

#endif // YOLO_H
