/*
 * image_interface.c
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/image_interface.h"

static float get_pixel(image m, int x, int y, int c)
{
    assert(x < m.w && y < m.h && c < m.c);
    return m.data[c*m.h*m.w + y*m.w + x];
}

image **load_alphabet_with_file(char *datafile) {
  int i, j;
  const int nsize = 8;
  image **alphabets = (image**)calloc(nsize, sizeof(image));
  char const * labels = "/labels/%d_%d.png";
  char * files = (char *) malloc(1 + strlen(datafile)+ strlen(labels) );
  strcpy(files, datafile);
  strcat(files, labels);
  for(j = 0; j < nsize; ++j){
    alphabets[j] = (image *)calloc(128, sizeof(image));
    for(i = 32; i < 127; ++i){
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0);
    }
  }
  return alphabets;
}

#ifdef OPENCV
void generate_image(image p, cv::Mat disp)
{
    int x,y,k;
    if(p.c == 3) rgbgr_image(p);
    //normalize_image(copy);

    int step = disp.step[0];
    for(y = 0; y < p.h; ++y){
        for(x = 0; x < p.w; ++x){
            for(k= 0; k < p.c; ++k){
                disp.data[y*step + x*p.c + k] = (unsigned char)(get_pixel(p,x,y,k)*255);
            }
        }
    }
}

image mat_to_image_(cv::Mat m)
{
    int h = m.rows;
    int w = m.cols;
    int c = m.channels();
    image im = make_image(w, h, c);
    return im;
}

image* mat_to_image_(cv::Mat m, image* im)
{
    int h = m.rows;
    int w = m.cols;
    int c = m.channels();
    unsigned char *data = (unsigned char *)m.data;
    int step = m.step;

    for(int i = 0; i < h; ++i){
        for(int k= 0; k < c; ++k){
            for(int j = 0; j < w; ++j){
                im->data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
  return im;
}	
#endif
