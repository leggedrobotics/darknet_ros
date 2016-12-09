cmake_minimum_required(VERSION 2.8.12)
project(darknet_rsl)

FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
	find_package(CUDA REQUIRED)
	message(STATUS "CUDA Version: " ${CUDA_VERSION_STRINGS})
	message(STATUS "CUDA Libararies: " ${CUDA_LIBRARIES})
	include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} ${CUDA_cusparse_LIBRARY})
	list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O2;-Xcompiler \"-fPIC\" ")
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_20,code=compute_20 ")
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=compute_30 ")
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_35,code=compute_35 ")
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=compute_50 ")
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=compute_52 ")
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	add_definitions(-DGPU)
	cuda_include_directories(src)
else()
	list(APPEND LIBRARIES "m")
endif()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	roscpp
	rospy
	std_msgs
	image_transport
	message_generation
)

add_message_files(
	FILES
	bbox.msg
	bbox_array.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

add_definitions(-DOPENCV)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	src
	${catkin_INCLUDE_DIRS}
)

if (CUDA_FOUND)
	link_directories(
  		/usr/local/cuda/lib64
	)
	
	cuda_add_executable(ros_interface 
	  src/ros_interface.cpp
	
	  src/gemm.c 			src/utils.c 
	  src/cuda.c 			src/deconvolutional_layer.c 
	  src/convolutional_layer.c	src/list.c 
	  src/image.c 			src/activations.c 
	  src/im2col.c 			src/col2im.c 
	  src/blas.c 			src/crop_layer.c 
	  src/dropout_layer.c 		src/maxpool_layer.c 
	  src/softmax_layer.c 		src/data.c 
	  src/matrix.c 			src/network.c 
	  src/connected_layer.c 	src/cost_layer.c 
	  src/parser.c 			src/option_list.c 
	  src/detection_layer.c		src/tag.c 
	  src/imagenet.c 		src/captcha.c 
	  src/route_layer.c 		src/writing.c 
	  src/box.c 			src/nightmare.c 
	  src/normalization_layer.c 	src/avgpool_layer.c 
	  src/coco.c 			src/dice.c 
	  src/yolo.c 			src/layer.c 
	  src/compare.c 		src/classifier.c  
	  src/local_layer.c 		src/shortcut_layer.c 
	  src/activation_layer.c	src/cifar.c
	  src/coco_demo.c		src/swag.c
	  src/crnn_layer.c		src/go.c
	  src/rnn.c			src/rnn_layer.c
	  src/rnn_vid.c			
	
	  src/convolutional_kernels.cu  src/deconvolutional_kernels.cu 
	  src/activation_kernels.cu 	src/im2col_kernels.cu 
	  src/col2im_kernels.cu 	src/blas_kernels.cu 
	  src/crop_layer_kernels.cu 	src/dropout_layer_kernels.cu 
	  src/maxpool_layer_kernels.cu  src/softmax_layer_kernels.cu 
	  src/network_kernels.cu 	src/avgpool_layer_kernels.cu 
	  src/yolo_kernels.cu
	)

	cuda_add_executable(${PROJECT_NAME}
	  src/YoloObjectDetector.cpp
	  src/yolo_object_detector_node.cpp
	
	  src/gemm.c                    src/utils.c
	  src/cuda.c                    src/deconvolutional_layer.c
	  src/convolutional_layer.c     src/list.c
	  src/image.c                   src/activations.c
	  src/im2col.c                  src/col2im.c
	  src/blas.c                    src/crop_layer.c
	  src/dropout_layer.c           src/maxpool_layer.c
	  src/softmax_layer.c           src/data.c
	  src/matrix.c                  src/network.c
	  src/connected_layer.c         src/cost_layer.c
	  src/parser.c                  src/option_list.c
	  src/detection_layer.c         src/tag.c
	  src/imagenet.c                src/captcha.c
	  src/route_layer.c             src/writing.c
	  src/box.c                     src/nightmare.c
	  src/normalization_layer.c     src/avgpool_layer.c
	  src/yolo_obj_detector.c       src/layer.c
	  src/compare.c                 src/classifier.c
	  src/local_layer.c             src/shortcut_layer.c
	  src/activation_layer.c        src/cifar.c
	  src/crnn_layer.c              src/go.c
	  src/rnn.c                     src/rnn_layer.c
	  src/rnn_vid.c			
	
	  src/convolutional_kernels.cu  src/deconvolutional_kernels.cu
	  src/activation_kernels.cu     src/im2col_kernels.cu
	  src/col2im_kernels.cu         src/blas_kernels.cu
	  src/crop_layer_kernels.cu     src/dropout_layer_kernels.cu
	  src/maxpool_layer_kernels.cu  src/softmax_layer_kernels.cu
	  src/network_kernels.cu        src/avgpool_layer_kernels.cu
	  src/yolo_kernels_ROSobj_detector.cu
	)

	target_link_libraries(ros_interface
	   m
	   pthread
	   stdc++
	   cuda 
	   cudart 
	   cublas 
	   curand
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
	   ${catkin_LIBRARIES}
	)
else()
	add_executable(ros_interface 
	  src/ros_interface.cpp
	
	  src/gemm.c 			src/utils.c 
	  src/cuda.c 			src/deconvolutional_layer.c 
	  src/convolutional_layer.c 	src/list.c 
	  src/image.c 			src/activations.c 
	  src/im2col.c 			src/col2im.c 
	  src/blas.c 			src/crop_layer.c 
	  src/dropout_layer.c 		src/maxpool_layer.c 
	  src/softmax_layer.c 		src/data.c 
	  src/matrix.c 			src/network.c 
	  src/connected_layer.c 	src/cost_layer.c 
	  src/parser.c 			src/option_list.c 
	  src/detection_layer.c		src/tag.c 
	  src/imagenet.c 		src/captcha.c 
	  src/route_layer.c 		src/writing.c 
	  src/box.c 			src/nightmare.c 
	  src/normalization_layer.c 	src/avgpool_layer.c 
	  src/coco.c 			src/dice.c 
	  src/yolo.c 			src/layer.c 
	  src/compare.c 		src/classifier.c  
	  src/local_layer.c 		src/shortcut_layer.c 
	  src/activation_layer.c	src/cifar.c
	  src/coco_demo.c		src/swag.c
	  src/crnn_layer.c		src/go.c
	  src/rnn.c			src/rnn_layer.c
	  src/rnn_vid.c			src/yolo_demo.cpp
	)
	
	add_executable(${PROJECT_NAME}
	  src/YoloObjectDetector.cpp
	  src/yolo_object_detector_node.cpp
	
	  src/gemm.c                    src/utils.c
	  src/cuda.c                    src/deconvolutional_layer.c
	  src/convolutional_layer.c     src/list.c
	  src/image.c                   src/activations.c
	  src/im2col.c                  src/col2im.c
	  src/blas.c                    src/crop_layer.c
	  src/dropout_layer.c           src/maxpool_layer.c
	  src/softmax_layer.c           src/data.c
	  src/matrix.c                  src/network.c
	  src/connected_layer.c         src/cost_layer.c
	  src/parser.c                  src/option_list.c
	  src/detection_layer.c         src/tag.c
	  src/imagenet.c                src/captcha.c
	  src/route_layer.c             src/writing.c
	  src/box.c                     src/nightmare.c
	  src/normalization_layer.c     src/avgpool_layer.c
	  src/yolo_obj_detector.c       src/layer.c
	  src/compare.c                 src/classifier.c
	  src/local_layer.c             src/shortcut_layer.c
	  src/activation_layer.c        src/cifar.c
	  src/crnn_layer.c              src/go.c
	  src/rnn.c                     src/rnn_layer.c
	  src/rnn_vid.c			src/yolo_demo_object_detector.cpp
	)

	target_link_libraries(ros_interface
	   m
	   pthread
	   stdc++
	   ${catkin_LIBRARIES}
	)

	target_link_libraries(${PROJECT_NAME}
	   m
	   pthread
	   stdc++
	   ${catkin_LIBRARIES}
	)

endif()
