/*
 * image_interface.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "image.h"
#include "opencv2/opencv.hpp"

static float get_pixel(image m, int x, int y, int c);
image** load_alphabet_with_file(char* datafile);
void generate_image(image p, cv::Mat& disp);

#endif
