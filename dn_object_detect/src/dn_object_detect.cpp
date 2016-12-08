//
//  dn_object_detect.cpp
//  pr2_perception
//
//  Created by Xun Wang on 12/05/16.
//  Copyright (c) 2016 Xun Wang. All rights reserved.
//

#include <ros/ros.h>
#include "MultiClassObjectDetector.h"

using namespace uts_perp;

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "dn_object_detect" );

  MultiClassObjectDetector dnDetector;

  dnDetector.init();

  dnDetector.continueProcessing();

  dnDetector.fini();

  return 0;
}



