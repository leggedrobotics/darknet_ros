cmake_minimum_required(VERSION 3.5)
project(darknet_ros)

include(CMakeDependentOption)

set(CMAKE_CXX_STANDARD 17)

# USER SETTINGS =================================================================

set(CUDA_ENABLE ON)
set(CUDNN_ENABLE ON) #  set(CUDNN_ENABLE ON)
set(FP16_ENABLE ON)

set(DOWNLOAD_YOLOV2_TINY OFF)
set(DOWNLOAD_YOLOV3 OFF)
set(DOWNLOAD_YOLOV4 OFF)
set(DOWNLOAD_YOLOV4_CSP ON)
set(DOWNLOAD_YOLOV4_TINY ON)
set(DOWNLOAD_YOLOV4_MISH OFF)
set(DOWNLOAD_YOLOV7_TINY ON)

# Jetson AGX Xavier & TITAN V is under version of Turing. So, please set manually.
# Turing or Ampere GPUs, FP16 is automatically enabled.

# set(CMAKE_CUDA_ARCHITECTURES 72)


# Define path of darknet folder here. ===========================================
find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../darknet/")
message(STATUS "Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Find CUDA ====================================================================
if(CUDA_ENABLE)
  find_package(CUDA REQUIRED)
endif()

if (CUDA_FOUND)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=[sm_61,compute_61]
    -gencode arch=compute_72,code=[sm_72,compute_72] # Jetson AGX Xavier or TITAN V
    -gencode arch=compute_75,code=[sm_75,compute_75]
    -gencode arch=compute_80,code=[sm_80,compute_80]
    -gencode arch=compute_86,code=[sm_86,compute_86] # GeForce RTX 3000 Series
    -gencode arch=compute_87,code=[sm_87,compute_87]
  )
  add_definitions(-DGPU)
endif()

# Find CUDNN =======================================================================

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../darknet/cmake/Modules/" ${CMAKE_MODULE_PATH})

if(CUDNN_ENABLE)
  find_package(CUDNN)
  message(STATUS "cuDNN Version: ${CUDNN_VERSION_STRINGS}")
  message(STATUS "cuDNN Libararies: ${CUDNN_LIBRARIES}")
endif()

if(CUDNN_FOUND)
    set(ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DCUDNN")
endif()

execute_process (
    COMMAND bash -c "lspci | grep NVIDIA | grep -e TU106 -e TU104 -e TU102 -e GA106 -e GA104 -e GA102"
    OUTPUT_VARIABLE outVar
)
message ( STATUS " GPU (FP16): " ${outVar} )
if(outVar AND FP16_ENABLE)
  set(CMAKE_CUDA_ARCHITECTURES 75)
endif()

if ( 70 IN_LIST CMAKE_CUDA_ARCHITECTURES OR
     72 IN_LIST CMAKE_CUDA_ARCHITECTURES OR
     75 IN_LIST CMAKE_CUDA_ARCHITECTURES OR
     80 IN_LIST CMAKE_CUDA_ARCHITECTURES OR
     86 IN_LIST CMAKE_CUDA_ARCHITECTURES)
  set(ENABLE_CUDNN_HALF "TRUE" CACHE BOOL "Enable CUDNN Half precision" FORCE)
  message(STATUS "Your setup supports half precision (CUDA_ARCHITECTURES >= 70)")
else()
  set(ENABLE_CUDNN_HALF "FALSE" CACHE BOOL "Enable CUDNN Half precision" FORCE)
  message(STATUS "Your setup does not support half precision (it requires CUDA_ARCHITECTURES >= 70)")
endif()

# else()
#   list(APPEND LIBRARIES "m")
# endif()

# Find X11 ============================================================================
message ( STATUS "Searching for X11..." )
find_package ( X11 REQUIRED )
if ( X11_FOUND )
  include_directories ( ${X11_INCLUDE_DIR} )
  link_libraries ( ${X11_LIBRARIES} )
  message ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  message ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
endif ( X11_FOUND )

# Find rquired packeges
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
    rclcpp
    rclpy
    rclcpp_action
    std_msgs
    image_transport
    cv_bridge
    sensor_msgs
    darknet_ros_msgs
    ament_index_cpp
)

# Enable OPENCV in darknet
# add_definitions(-DOPENCV)
add_definitions(-O4 -g)

include_directories(
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include
  ${DARKNET_PATH}/3rdparty/stb/include
  include
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
    src/YoloObjectDetector.cpp                    src/image_interface.cpp
    src/yolo_object_detector_node.cpp
)

set(DARKNET_CORE_FILES
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/activations.c
    ${DARKNET_PATH}/src/art.c                     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/blas.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/captcha.c
    ${DARKNET_PATH}/src/cifar.c                   ${DARKNET_PATH}/src/classifier.c
    ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/col2im.c
    ${DARKNET_PATH}/src/compare.c                 ${DARKNET_PATH}/src/connected_layer.c
    ${DARKNET_PATH}/src/conv_lstm_layer.c         ${DARKNET_PATH}/src/convolutional_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/cpu_gemm.c
    ${DARKNET_PATH}/src/crnn_layer.c              ${DARKNET_PATH}/src/crop_layer.c
    ${DARKNET_PATH}/src/dark_cuda.c               ${DARKNET_PATH}/src/darknet.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/deconvolutional_layer.c
    ${DARKNET_PATH}/src/demo.c                    ${DARKNET_PATH}/src/detection_layer.c
    ${DARKNET_PATH}/src/detector.c                ${DARKNET_PATH}/src/dice.c
    ${DARKNET_PATH}/src/dropout_layer.c           ${DARKNET_PATH}/src/gaussian_yolo_layer.c
    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/getopt.c
    ${DARKNET_PATH}/src/gettimeofday.c            ${DARKNET_PATH}/src/go.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/http_stream.cpp
    ${DARKNET_PATH}/src/im2col.c                  ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/image_opencv.cpp          ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/list.c                    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/lstm_layer.c              ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/maxpool_layer.c           ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/nightmare.c               ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/option_list.c             ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/region_layer.c            ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/reorg_old_layer.c         ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/rnn_layer.c               ${DARKNET_PATH}/src/rnn_vid.c
    ${DARKNET_PATH}/src/route_layer.c             ${DARKNET_PATH}/src/sam_layer.c
    ${DARKNET_PATH}/src/scale_channels_layer.c    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/softmax_layer.c           ${DARKNET_PATH}/src/super.c
    ${DARKNET_PATH}/src/swag.c                    ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/tree.c                    ${DARKNET_PATH}/src/upsample_layer.c
    ${DARKNET_PATH}/src/utils.c                   ${DARKNET_PATH}/src/voxel.c
    ${DARKNET_PATH}/src/writing.c                 ${DARKNET_PATH}/src/yolo.c
    ${DARKNET_PATH}/src/yolo_layer.c              ${DARKNET_PATH}/src/representation_layer.c

    
)

set(DARKNET_CUDA_FILES
    ${DARKNET_PATH}/src/activation_kernels.cu       ${DARKNET_PATH}/src/avgpool_layer_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu             ${DARKNET_PATH}/src/col2im_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu    ${DARKNET_PATH}/src/crop_layer_kernels.cu
    ${DARKNET_PATH}/src/deconvolutional_kernels.cu  ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/im2col_kernels.cu           ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
    ${DARKNET_PATH}/src/network_kernels.cu
)

set_source_files_properties(${PROJECT_LIB_FILES} PROPERTIES LANGUAGE CXX)

if (CUDA_FOUND)

  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  )

  cuda_add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
    ${DARKNET_CUDA_FILES}
  )
  
  ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

  if(CUDNN_FOUND)
    if(ENABLE_CUDNN_HALF)
      target_link_libraries(${PROJECT_NAME}_lib
        cuda
        cudart
        cublas
        curand
        CuDNN::CuDNN
      )
      target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
        -DCUDNN
        -DCUDNN_HALF
      )
    else()
      target_link_libraries(${PROJECT_NAME}_lib
        cuda
        cudart
        cublas
        curand
        CuDNN::CuDNN
      )
      target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
        -DCUDNN
      )
    endif()
  else()
    target_link_libraries(${PROJECT_NAME}_lib
      cuda
      cudart
      cublas
      curand
    )
  endif()

  cuda_add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
  )
  
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

else()

  add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
    ${DARKNET_CORE_FILES}
  )
  ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE -DOPENCV)

  add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
  )
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

endif()

find_package(OpenMP)

if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

target_link_libraries(${PROJECT_NAME}_lib
  m
  pthread
  stdc++
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE -DOPENCV)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)
target_compile_definitions(${PROJECT_NAME} PRIVATE -DOPENCV)

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

# Download yolov2-tiny.weights
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")

if(DOWNLOAD_YOLOV2_TINY)
  set(FILE "${PATH}/yolov2-tiny.weights")
  message(STATUS "Checking and downloading yolov2-tiny.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    # yolov2-tiny.cfg is already in the repo.
    execute_process(COMMAND wget http://pjreddie.com/media/files/yolov2-tiny.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

# Download yolov3.weights
if(DOWNLOAD_YOLOV3)
  set(FILE "${PATH}/yolov3.weights")
  message(STATUS "Checking and downloading yolov3.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    # yolov3.cfg is already in the repo.
    execute_process(COMMAND wget http://pjreddie.com/media/files/yolov3.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

# Download yolov4.weights
if(DOWNLOAD_YOLOV4)
  set(FILE "${PATH}/yolov4.weights")
  message(STATUS "Checking and downloading yolov4.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget -q https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

# Download yolov4-tiny.weights
if(DOWNLOAD_YOLOV4_TINY)
  set(FILE "${PATH}/yolov4-tiny.weights")
  message(STATUS "Checking and downloading yolov4-tiny.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    # yolov4.cfg is already in the repo.
    execute_process(COMMAND wget -q https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

# Download yolov4-csp.weights
if(DOWNLOAD_YOLOV4_CSP)
  set(FILE "${PATH}/yolov4-csp.weights")
  message(STATUS "Checking and downloading yolov4-csp.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget -q https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-csp.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

# Download yolov4-mish.weights

if(DOWNLOAD_YOLOV4_MISH)
  set(FILE "${PATH}/yolov4-mish.weights")
  message(STATUS "Checking and downloading yolov4-mish.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget -q https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4x-mish.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

if(DOWNLOAD_YOLOV7_TINY)
  set(FILE "${PATH}/yolov7-tiny.weights")
  message(STATUS "Checking and downloading yolov7-tiny.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget -q https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov7-tiny.weights -P ${PATH})
  endif()
  message(STATUS "Done.")
endif()

install(DIRECTORY yolo_network_config/cfg yolo_network_config/weights DESTINATION share/${PROJECT_NAME}/yolo_network_config/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

#############
## Testing ##
#############

# if(BUILD_TESTING)
#   # Download yolov2.weights
#   set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
#   set(FILE "${PATH}/yolov2.weights")
#   message(STATUS "Checking and downloading yolov2.weights if needed ...")
#   if (NOT EXISTS "${FILE}")
#     message(STATUS "... file does not exist. Downloading now ...")
#     execute_process(COMMAND wget http://pjreddie.com/media/files/yolov2.weights -P ${PATH})
#   endif()

#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)

  #ament_add_gtest(${PROJECT_NAME}_object_detection-test
  #  test/object_detection.test
  #  test/test_main.cpp
  #  test/ObjectDetection.cpp
  #)
  #target_link_libraries(${PROJECT_NAME}_object_detection-test
  #  ${PROJECT_NAME}_lib
  #)
# endif()


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${dependencies})

ament_package()
