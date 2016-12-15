cmake_minimum_required(VERSION 2.8.12)
project(darknet2_rsl)

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
		~/git/darknet/src
		src
		${catkin_INCLUDE_DIRS}
	)

	link_directories(
  		/usr/local/cuda/lib64
	)
	
	cuda_add_executable(ros_interface 
		src/ros_interface.cpp
		src/yolo_demo.cpp
		src/image_interface.c
		
		~/git/darknet/src/gemm.c 					~/git/darknet/src/utils.c
		~/git/darknet/src/cuda.c					~/git/darknet/src/convolutional_layer.c 
		~/git/darknet/src/list.c 					~/git/darknet/src/image.c
		~/git/darknet/src/activations.c			~/git/darknet/src/im2col.c
		~/git/darknet/src/col2im.c 				~/git/darknet/src/blas.c 
		~/git/darknet/src/crop_layer.c 			~/git/darknet/src/dropout_layer.c
		~/git/darknet/src/maxpool_layer.c			~/git/darknet/src/softmax_layer.c
		~/git/darknet/src/data.c					~/git/darknet/src/matrix.c
		~/git/darknet/src/network.c 				~/git/darknet/src/connected_layer.c
		~/git/darknet/src/cost_layer.c			~/git/darknet/src/parser.c
		~/git/darknet/src/option_list.c			~/git/darknet/src/tree.c 
		~/git/darknet/src/detection_layer.c 		~/git/darknet/src/captcha.c
		~/git/darknet/src/route_layer.c 			~/git/darknet/src/writing.c
		~/git/darknet/src/box.c					~/git/darknet/src/nightmare.c
		~/git/darknet/src/normalization_layer.c 	~/git/darknet/src/avgpool_layer.c 
		~/git/darknet/src/coco.c 					~/git/darknet/src/dice.c 
		~/git/darknet/src/yolo.c 					~/git/darknet/src/demo.c
		~/git/darknet/src/layer.c					~/git/darknet/src/compare.c 
		~/git/darknet/src/classifier.c 			~/git/darknet/src/local_layer.c
		~/git/darknet/src/swag.c 					~/git/darknet/src/shortcut_layer.c 
		~/git/darknet/src/activation_layer.c		~/git/darknet/src/rnn_layer.c 
		~/git/darknet/src/gru_layer.c				~/git/darknet/src/rnn.c
		~/git/darknet/src/rnn_vid.c				~/git/darknet/src/crnn_layer.c
		~/git/darknet/src/voxel.c					~/git/darknet/src/tag.c 
		~/git/darknet/src/cifar.c 				~/git/darknet/src/go.c 
		~/git/darknet/src/batchnorm_layer.c 		~/git/darknet/src/art.c
		~/git/darknet/src/region_layer.c 			~/git/darknet/src/reorg_layer.c 
		~/git/darknet/src/super.c	 	
	
		~/git/darknet/src/convolutional_kernels.cu	~/git/darknet/src/network_kernels.cu
		~/git/darknet/src/activation_kernels.cu 		~/git/darknet/src/im2col_kernels.cu 
		~/git/darknet/src/col2im_kernels.cu 			~/git/darknet/src/blas_kernels.cu 
		~/git/darknet/src/crop_layer_kernels.cu 		~/git/darknet/src/dropout_layer_kernels.cu 
		~/git/darknet/src/maxpool_layer_kernels.cu	~/git/darknet/src/avgpool_layer_kernels.cu	
	)

	cuda_add_executable(${PROJECT_NAME}
		src/YoloObjectDetector.cpp
		src/yolo_object_detector_node.cpp
		src/yolo_demo_object_detector.cpp
		src/image_interface.c
		
		~/git/darknet/src/gemm.c 					~/git/darknet/src/utils.c
		~/git/darknet/src/cuda.c					~/git/darknet/src/convolutional_layer.c 
		~/git/darknet/src/list.c 					~/git/darknet/src/image.c
		~/git/darknet/src/activations.c			~/git/darknet/src/im2col.c
		~/git/darknet/src/col2im.c 				~/git/darknet/src/blas.c 
		~/git/darknet/src/crop_layer.c 			~/git/darknet/src/dropout_layer.c
		~/git/darknet/src/maxpool_layer.c			~/git/darknet/src/softmax_layer.c
		~/git/darknet/src/data.c					~/git/darknet/src/matrix.c
		~/git/darknet/src/network.c 				~/git/darknet/src/connected_layer.c
		~/git/darknet/src/cost_layer.c			~/git/darknet/src/parser.c
		~/git/darknet/src/option_list.c			~/git/darknet/src/tree.c 
		~/git/darknet/src/detection_layer.c 		~/git/darknet/src/captcha.c
		~/git/darknet/src/route_layer.c 			~/git/darknet/src/writing.c
		~/git/darknet/src/box.c					~/git/darknet/src/nightmare.c
		~/git/darknet/src/normalization_layer.c 	~/git/darknet/src/avgpool_layer.c 
		~/git/darknet/src/coco.c 					~/git/darknet/src/dice.c 
		~/git/darknet/src/yolo.c 					~/git/darknet/src/demo.c
		~/git/darknet/src/layer.c					~/git/darknet/src/compare.c 
		~/git/darknet/src/classifier.c 			~/git/darknet/src/local_layer.c
		~/git/darknet/src/swag.c 					~/git/darknet/src/shortcut_layer.c 
		~/git/darknet/src/activation_layer.c		~/git/darknet/src/rnn_layer.c 
		~/git/darknet/src/gru_layer.c				~/git/darknet/src/rnn.c
		~/git/darknet/src/rnn_vid.c				~/git/darknet/src/crnn_layer.c
		~/git/darknet/src/voxel.c					~/git/darknet/src/tag.c 
		~/git/darknet/src/cifar.c 				~/git/darknet/src/go.c 
		~/git/darknet/src/batchnorm_layer.c 		~/git/darknet/src/art.c
		~/git/darknet/src/region_layer.c 			~/git/darknet/src/reorg_layer.c 
		~/git/darknet/src/super.c	 	
		
		~/git/darknet/src/convolutional_kernels.cu	~/git/darknet/src/network_kernels.cu
		~/git/darknet/src/activation_kernels.cu 		~/git/darknet/src/im2col_kernels.cu 
		~/git/darknet/src/col2im_kernels.cu 			~/git/darknet/src/blas_kernels.cu 
		~/git/darknet/src/crop_layer_kernels.cu 		~/git/darknet/src/dropout_layer_kernels.cu 
		~/git/darknet/src/maxpool_layer_kernels.cu	~/git/darknet/src/avgpool_layer_kernels.cu	
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
		~/git/darknet/src
		src
		${catkin_INCLUDE_DIRS}
	)

	add_executable(ros_interface 
		src/ros_interface.cpp
		src/yolo_demo.cpp
		src/image_interface.c
		
		~/git/darknet/src/gemm.c 					~/git/darknet/src/utils.c
		~/git/darknet/src/cuda.c					~/git/darknet/src/convolutional_layer.c 
		~/git/darknet/src/list.c 					~/git/darknet/src/image.c
		~/git/darknet/src/activations.c			~/git/darknet/src/im2col.c
		~/git/darknet/src/col2im.c 				~/git/darknet/src/blas.c 
		~/git/darknet/src/crop_layer.c 			~/git/darknet/src/dropout_layer.c
		~/git/darknet/src/maxpool_layer.c			~/git/darknet/src/softmax_layer.c
		~/git/darknet/src/data.c					~/git/darknet/src/matrix.c
		~/git/darknet/src/network.c 				~/git/darknet/src/connected_layer.c
		~/git/darknet/src/cost_layer.c			~/git/darknet/src/parser.c
		~/git/darknet/src/option_list.c			~/git/darknet/src/tree.c 
		~/git/darknet/src/detection_layer.c 		~/git/darknet/src/captcha.c
		~/git/darknet/src/route_layer.c 			~/git/darknet/src/writing.c
		~/git/darknet/src/box.c					~/git/darknet/src/nightmare.c
		~/git/darknet/src/normalization_layer.c 	~/git/darknet/src/avgpool_layer.c 
		~/git/darknet/src/coco.c 					~/git/darknet/src/dice.c 
		~/git/darknet/src/yolo.c 					~/git/darknet/src/demo.c
		~/git/darknet/src/layer.c					~/git/darknet/src/compare.c 
		~/git/darknet/src/classifier.c 			~/git/darknet/src/local_layer.c
		~/git/darknet/src/swag.c 					~/git/darknet/src/shortcut_layer.c 
		~/git/darknet/src/activation_layer.c		~/git/darknet/src/rnn_layer.c 
		~/git/darknet/src/gru_layer.c				~/git/darknet/src/rnn.c
		~/git/darknet/src/rnn_vid.c				~/git/darknet/src/crnn_layer.c
		~/git/darknet/src/voxel.c					~/git/darknet/src/tag.c 
		~/git/darknet/src/cifar.c 				~/git/darknet/src/go.c 
		~/git/darknet/src/batchnorm_layer.c 		~/git/darknet/src/art.c
		~/git/darknet/src/region_layer.c 			~/git/darknet/src/reorg_layer.c 
		~/git/darknet/src/super.c	 	
	)
	
	add_executable(${PROJECT_NAME}
		src/YoloObjectDetector.cpp
		src/yolo_object_detector_node.cpp
		src/yolo_demo_object_detector.cpp
		src/image_interface.c
	
		~/git/darknet/src/gemm.c 					~/git/darknet/src/utils.c
		~/git/darknet/src/cuda.c					~/git/darknet/src/convolutional_layer.c 
		~/git/darknet/src/list.c 					~/git/darknet/src/image.c
		~/git/darknet/src/activations.c			~/git/darknet/src/im2col.c
		~/git/darknet/src/col2im.c 				~/git/darknet/src/blas.c 
		~/git/darknet/src/crop_layer.c 			~/git/darknet/src/dropout_layer.c
		~/git/darknet/src/maxpool_layer.c			~/git/darknet/src/softmax_layer.c
		~/git/darknet/src/data.c					~/git/darknet/src/matrix.c
		~/git/darknet/src/network.c 				~/git/darknet/src/connected_layer.c
		~/git/darknet/src/cost_layer.c			~/git/darknet/src/parser.c
		~/git/darknet/src/option_list.c			~/git/darknet/src/tree.c 
		~/git/darknet/src/detection_layer.c 		~/git/darknet/src/captcha.c
		~/git/darknet/src/route_layer.c 			~/git/darknet/src/writing.c
		~/git/darknet/src/box.c					~/git/darknet/src/nightmare.c
		~/git/darknet/src/normalization_layer.c 	~/git/darknet/src/avgpool_layer.c 
		~/git/darknet/src/coco.c 					~/git/darknet/src/dice.c 
		~/git/darknet/src/yolo.c 					~/git/darknet/src/demo.c
		~/git/darknet/src/layer.c					~/git/darknet/src/compare.c 
		~/git/darknet/src/classifier.c 			~/git/darknet/src/local_layer.c
		~/git/darknet/src/swag.c 					~/git/darknet/src/shortcut_layer.c 
		~/git/darknet/src/activation_layer.c		~/git/darknet/src/rnn_layer.c 
		~/git/darknet/src/gru_layer.c				~/git/darknet/src/rnn.c
		~/git/darknet/src/rnn_vid.c				~/git/darknet/src/crnn_layer.c
		~/git/darknet/src/voxel.c					~/git/darknet/src/tag.c 
		~/git/darknet/src/cifar.c 				~/git/darknet/src/go.c 
		~/git/darknet/src/batchnorm_layer.c 		~/git/darknet/src/art.c
		~/git/darknet/src/region_layer.c 			~/git/darknet/src/reorg_layer.c 
		~/git/darknet/src/super.c  	
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
