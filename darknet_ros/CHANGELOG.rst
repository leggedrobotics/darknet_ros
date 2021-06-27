^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package darknet_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5 (2021-04-08)
------------------
* Merge pull request `#308 <https://github.com/leggedrobotics/darknet_ros/issues/308>`_ from leggedrobotics/noetic
  Noetic
* Minor fix in CMake.
* Minor fix in test.
* Updated readme.
* Updated to newest darknet and fixed opencv on Ubuntu 20.04.
* First changes in CMakeLists.
* Merge pull request `#287 <https://github.com/leggedrobotics/darknet_ros/issues/287>`_ from leggedrobotics/feature/nodeletize
  Feature/nodeletize
* Small adjustments
* Added pointer deletion in destructor and minor formatting
* Minor formatting
* Adding option to use nodelet
* Add libxext
* Add libxt-dev
* Retrieve binaries from Github releases
* Merge pull request `#232 <https://github.com/leggedrobotics/darknet_ros/issues/232>`_ from leggedrobotics/add-required-deps
  Update package.xml dependencies
* Don't require cmake-clang-tools
* Update package.xml dependencies
* Added clang tooling.
* Merge pull request `#207 <https://github.com/leggedrobotics/darknet_ros/issues/207>`_ from umdlife/install_weights
  Add install targets for configuration files
* Add install targets for configuration files
  Adds the `launch`, `config`, and `yolo_network_config` folders to the
  install target for `darknet_ros` so they are available in the catkin
  install directory.
* Fixed linking of cuda libraries.
* Merge branch 'kunaltyagi-cleanup'
* Cleaned up CMakeLists.txt, used OpenCV C++ API for cpp file
* Merge pull request `#189 <https://github.com/leggedrobotics/darknet_ros/issues/189>`_ from martinspedro/master
  /darknet_ros/found_object uses custom msg with Header for improving synchronization
* YOLO publishes Object Count with Time stamp using custom msg with Header
* Merge pull request `#183 <https://github.com/leggedrobotics/darknet_ros/issues/183>`_ from kunaltyagi/id_num
  Add numerical ID and launch param for image
* Adding numerical ID and launch param for image
* Merge pull request `#182 <https://github.com/leggedrobotics/darknet_ros/issues/182>`_ from leggedrobotics/fix/image_publisher
  Fixed copy of image publisher.
* Fixed copy of image publisher.
* Merge branch 'kjbilton-master'
* Increased scope of image acquisition mutex to prevent image overwriting
* Merge branch 'utra-robosoccer-master'
* Removed warnings caused by catkin builds
* Added test_depend of wget to accomodate Jenkins.
* Contributors: Jason Wang, Kunal Tyagi, Kyle Bilton, Marko Bjelonic, Pedro Martins, Tom Lankhorst, Tomas Gareau, timonh

1.1.4 (2019-03-03)
------------------
* Merge pull request `#141 <https://github.com/leggedrobotics/darknet_ros/issues/141>`_ from lorenwel/feature/launch_file_arg
  Added arg for launch file parameter files
* Fixed synatx error
* Removed unnecessary args
* Adapted yolo_v3.launch to new launch file
* Added launch file arguments for parameter files
* Merge branch 'Texas-Aerial-Robotics-headerFixForUpsteam'
* Merge branch 'headerFixForUpsteam' of https://github.com/Texas-Aerial-Robotics/darknet_ros into Texas-Aerial-Robotics-headerFixForUpsteam
* Remove unused variable
* Merge branch 'headerFixForUpsteam' of https://github.com/Texas-Aerial-Robotics/darknet_ros into Texas-Aerial-Robotics-headerFixForUpsteam
* Multithreading mismatched image header fix
* Forgot to add image.
* Cropped test image.
* Changed image for test.
* Changed resame image.
* Added new images for test.
* Removed twice loading of weightfile.
* Contributors: Lorenz Wellhausen, Marko Bjelonic, Umer Salman, lorenwel

1.1.3 (2018-04-26)
------------------
* Fixed iteration through detection boxes.
* Merge pull request `#80 <https://github.com/leggedrobotics/darknet_ros/issues/80>`_ from leggedrobotics/feature/yolo3
  Feature/yolo3
* Fixed publishers.
* Applied first changes for yolo v3.
* Updated darknet and added launch files for yolov3.
* Merge pull request `#73 <https://github.com/leggedrobotics/darknet_ros/issues/73>`_ from leggedrobotics/fix/weights
  Fix/weights
* Fixed weights.
* Fix test.
* Fixed formatting part 2.
* Fixed naming.
* Merge branch 'firephinx-master'
* Merge branch 'master' of https://github.com/firephinx/darknet_ros into firephinx-master
* Merge pull request `#62 <https://github.com/leggedrobotics/darknet_ros/issues/62>`_ from warp1337/master
  Reduced window size to reasonable values
* Reduced window size to reasonable values
* Added rgb_image_header to BoundingBoxes msg.
* Updated to the latest darknet version.
* Merge pull request `#57 <https://github.com/leggedrobotics/darknet_ros/issues/57>`_ from leggedrobotics/devel/threads
  Devel/threads
* Rearranged.
* Fixed action with new threads.
* Adapted package description.
* Added publisher.
* Merge branch 'master' into devel/threads
* Rearranged code.
* Update package.xml
* Fixed image_view if x11 is not running.
* COmment runYolo().
* Update object_detector_demo.cpp
* Changed ros config.
* Node is shutting down properly.
* Rearranged code and added threads.
* Contributors: Kevin Zhang, Marko Bjelonic, fl

1.1.2 (2018-01-06)
------------------
* First release of darknet_ros.
