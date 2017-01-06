cmake_minimum_required(VERSION 2.8.12)
project(darknet_rsl)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Define path of darknet folder here.
# set(DARKNET_PATH "/home/markob/git/darknet")
find_path(DARKNET_PATH 
	NAMES "README.md"
	HINTS "$ENV{HOME}/darknet/" "$ENV{HOME}/git/darknet/" "$ENV{HOME}/catkin_ws/src/darknet/" "$ENV{HOME}/workspace/darknet_rsl/source/dependecies/darknet/")
message("Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_61,code=sm_61
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED 
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    darknet_rsl_msgs
    image_transport
    param_io
)

add_definitions(-DOPENCV)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    darknet_rsl_msgs
    image_transport
    param_io
  DEPENDS
    boost
)

if (CUDA_FOUND)
  include_directories(
    ${DARKNET_PATH}/src
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )

  link_directories(
    /usr/local/cuda/lib64
  )

  cuda_add_executable(ros_interface
    src/ros_interface.cpp
    src/yolo_demo.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/convolutional_layer.c
    ${DARKNET_PATH}/src/list.c                    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/blas.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/dropout_layer.c
    ${DARKNET_PATH}/src/maxpool_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/network.c                 ${DARKNET_PATH}/src/connected_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/option_list.c             ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/captcha.c
    ${DARKNET_PATH}/src/route_layer.c             ${DARKNET_PATH}/src/writing.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/nightmare.c
    ${DARKNET_PATH}/src/normalization_layer.c     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/dice.c
    ${DARKNET_PATH}/src/yolo.c                    ${DARKNET_PATH}/src/demo.c
    ${DARKNET_PATH}/src/layer.c                   ${DARKNET_PATH}/src/compare.c
    ${DARKNET_PATH}/src/classifier.c              ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/swag.c                    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/rnn_vid.c                 ${DARKNET_PATH}/src/crnn_layer.c
    ${DARKNET_PATH}/src/voxel.c                   ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/cifar.c                   ${DARKNET_PATH}/src/go.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/art.c
    ${DARKNET_PATH}/src/region_layer.c            ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/super.c

    ${DARKNET_PATH}/src/convolutional_kernels.cu  ${DARKNET_PATH}/src/network_kernels.cu
    ${DARKNET_PATH}/src/activation_kernels.cu     ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         ${DARKNET_PATH}/src/blas_kernels.cu
    ${DARKNET_PATH}/src/crop_layer_kernels.cu     ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/maxpool_layer_kernels.cu  ${DARKNET_PATH}/src/avgpool_layer_kernels.cu
  )

  cuda_add_executable(${PROJECT_NAME}
    src/YoloObjectDetector.cpp
    src/yolo_object_detector_node.cpp
    src/object_detector_demo.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/convolutional_layer.c
    ${DARKNET_PATH}/src/list.c                    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/blas.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/dropout_layer.c
    ${DARKNET_PATH}/src/maxpool_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/network.c                 ${DARKNET_PATH}/src/connected_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/option_list.c             ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/captcha.c
    ${DARKNET_PATH}/src/route_layer.c             ${DARKNET_PATH}/src/writing.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/nightmare.c
    ${DARKNET_PATH}/src/normalization_layer.c     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/dice.c
    ${DARKNET_PATH}/src/yolo.c                    ${DARKNET_PATH}/src/demo.c
    ${DARKNET_PATH}/src/layer.c                   ${DARKNET_PATH}/src/compare.c
    ${DARKNET_PATH}/src/classifier.c              ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/swag.c                    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/rnn_vid.c                 ${DARKNET_PATH}/src/crnn_layer.c
    ${DARKNET_PATH}/src/voxel.c                   ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/cifar.c                   ${DARKNET_PATH}/src/go.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/art.c
    ${DARKNET_PATH}/src/region_layer.c            ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/super.c

    ${DARKNET_PATH}/src/convolutional_kernels.cu  ${DARKNET_PATH}/src/network_kernels.cu
    ${DARKNET_PATH}/src/activation_kernels.cu     ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         ${DARKNET_PATH}/src/blas_kernels.cu
    ${DARKNET_PATH}/src/crop_layer_kernels.cu     ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/maxpool_layer_kernels.cu  ${DARKNET_PATH}/src/avgpool_layer_kernels.cu
  )

  target_link_libraries(ros_interface
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  target_link_libraries(${PROJECT_NAME}
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else()
  include_directories(
    ${DARKNET_PATH}/src
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(ros_interface
    src/ros_interface.cpp
    src/yolo_demo.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/convolutional_layer.c
    ${DARKNET_PATH}/src/list.c                    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/blas.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/dropout_layer.c
    ${DARKNET_PATH}/src/maxpool_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/network.c                 ${DARKNET_PATH}/src/connected_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/option_list.c             ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/captcha.c
    ${DARKNET_PATH}/src/route_layer.c             ${DARKNET_PATH}/src/writing.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/nightmare.c
    ${DARKNET_PATH}/src/normalization_layer.c     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/dice.c
    ${DARKNET_PATH}/src/yolo.c                    ${DARKNET_PATH}/src/demo.c
    ${DARKNET_PATH}/src/layer.c                   ${DARKNET_PATH}/src/compare.c
    ${DARKNET_PATH}/src/classifier.c              ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/swag.c                    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/rnn_vid.c                 ${DARKNET_PATH}/src/crnn_layer.c
    ${DARKNET_PATH}/src/voxel.c                   ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/cifar.c                   ${DARKNET_PATH}/src/go.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/art.c
    ${DARKNET_PATH}/src/region_layer.c            ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/super.c
  )

  add_executable(${PROJECT_NAME}
    src/YoloObjectDetector.cpp
    src/yolo_object_detector_node.cpp
    src/object_detector_demo.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/convolutional_layer.c
    ${DARKNET_PATH}/src/list.c                    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/blas.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/dropout_layer.c
    ${DARKNET_PATH}/src/maxpool_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/network.c                 ${DARKNET_PATH}/src/connected_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/option_list.c             ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/captcha.c
    ${DARKNET_PATH}/src/route_layer.c             ${DARKNET_PATH}/src/writing.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/nightmare.c
    ${DARKNET_PATH}/src/normalization_layer.c     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/dice.c
    ${DARKNET_PATH}/src/yolo.c                    ${DARKNET_PATH}/src/demo.c
    ${DARKNET_PATH}/src/layer.c                   ${DARKNET_PATH}/src/compare.c
    ${DARKNET_PATH}/src/classifier.c              ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/swag.c                    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/rnn_vid.c                 ${DARKNET_PATH}/src/crnn_layer.c
    ${DARKNET_PATH}/src/voxel.c                   ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/cifar.c                   ${DARKNET_PATH}/src/go.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/art.c
    ${DARKNET_PATH}/src/region_layer.c            ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/super.c
  )

  target_link_libraries(ros_interface
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  target_link_libraries(${PROJECT_NAME}
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()
