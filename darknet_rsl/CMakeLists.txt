cmake_minimum_required(VERSION 2.8.12)
project(darknet_rsl)

FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
	find_package(CUDA REQUIRED)
	message(STATUS "CUDA Version: " ${CUDA_VERSION_STRINGS})
	message(STATUS "CUDA Libararies: " ${CUDA_LIBRARIES})
	set(
		CUDA_NVCC_FLAGS
		${CUDA_NVCC_FLAGS};
		-O3 -gencode arch=compute_61,code=sm_61
	)
	add_definitions(-DGPU)
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

if (CUDA_FOUND)
	include_directories(
		/usr/local/cuda/include
		src
		${catkin_INCLUDE_DIRS}
	)

	link_directories(
  		/usr/local/cuda/lib64
	)
	
	cuda_add_executable(ros_interface 
		src/ros_interface.cpp
		src/yolo_demo.cpp
	
		src/gemm.c 					src/utils.c 
		src/cuda.c 					src/deconvolutional_layer.c 
		src/convolutional_layer.c	src/list.c 
		src/image.c 				src/activations.c 
		src/im2col.c 				src/col2im.c 
		src/blas.c 					src/crop_layer.c 
		src/dropout_layer.c 		src/maxpool_layer.c 
		src/softmax_layer.c 		src/data.c 
		src/matrix.c 				src/network.c 
		src/connected_layer.c 		src/cost_layer.c 
		src/parser.c 				src/option_list.c 
		src/detection_layer.c		src/tag.c 
		src/imagenet.c 				src/captcha.c 
		src/route_layer.c 			src/writing.c 
		src/box.c 					src/nightmare.c 
		src/normalization_layer.c 	src/avgpool_layer.c 
		src/coco.c 					src/dice.c 
		src/yolo.c 					src/layer.c 
		src/compare.c 				src/classifier.c  
		src/local_layer.c 			src/shortcut_layer.c 
		src/activation_layer.c		src/cifar.c
		src/coco_demo.c				src/swag.c
		src/crnn_layer.c			src/go.c
		src/rnn.c					src/rnn_layer.c
		src/rnn_vid.c			
	
		src/convolutional_kernels.cu	src/deconvolutional_kernels.cu 
		src/activation_kernels.cu 		src/im2col_kernels.cu 
		src/col2im_kernels.cu 			src/blas_kernels.cu 
		src/crop_layer_kernels.cu 		src/dropout_layer_kernels.cu 
		src/maxpool_layer_kernels.cu	src/softmax_layer_kernels.cu 
		src/network_kernels.cu 			src/avgpool_layer_kernels.cu 
	)

	cuda_add_executable(${PROJECT_NAME}
		src/YoloObjectDetector.cpp
		src/yolo_object_detector_node.cpp
		src/yolo_demo_object_detector.cpp
	
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
	include_directories(
		src
		${catkin_INCLUDE_DIRS}
	)

	add_executable(ros_interface 
		src/ros_interface.cpp
		src/yolo_demo.cpp
		
		src/gemm.c 					src/utils.c 
		src/cuda.c 					src/deconvolutional_layer.c 
		src/convolutional_layer.c 	src/list.c 
		src/image.c 				src/activations.c 
		src/im2col.c 				src/col2im.c 
		src/blas.c 					src/crop_layer.c 
		src/dropout_layer.c 		src/maxpool_layer.c 
		src/softmax_layer.c 		src/data.c 
		src/matrix.c 				src/network.c 
		src/connected_layer.c 		src/cost_layer.c 
		src/parser.c 				src/option_list.c 
		src/detection_layer.c		src/tag.c 
		src/imagenet.c 				src/captcha.c 
		src/route_layer.c 			src/writing.c 
		src/box.c 					src/nightmare.c 
		src/normalization_layer.c 	src/avgpool_layer.c 
		src/coco.c 					src/dice.c 
		src/yolo.c 					src/layer.c 
		src/compare.c 				src/classifier.c  
		src/local_layer.c 			src/shortcut_layer.c 
		src/activation_layer.c		src/cifar.c
		src/coco_demo.c				src/swag.c
		src/crnn_layer.c			src/go.c
		src/rnn.c					src/rnn_layer.c
		src/rnn_vid.c			
	)
	
	add_executable(${PROJECT_NAME}
		src/YoloObjectDetector.cpp
		src/yolo_object_detector_node.cpp
		src/yolo_demo_object_detector.cpp
		
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
