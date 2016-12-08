cmake_minimum_required(VERSION 2.8)
project(yololib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMake")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
## Compiler settings

#thread
FIND_PACKAGE(Threads)
list(APPEND LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

# CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
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

#BOOST
FIND_PACKAGE(OpenCV REQUIRED)

add_definitions(-DOPENCV)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIR})

include_directories(src)
set(SRC_DIR src)

FILE(GLOB SRC_FILES ${SRC_DIR}/*.c)
FILE(GLOB CU_SRC_FILES ${SRC_DIR}/*.cu)
LIST(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/server.c")
LIST(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/old.c")
LIST(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/cpu_gemm.c")
LIST(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/darknet.c")
LIST(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/yolo_demo.c")

if (CUDA_FOUND)
message(STATUS "CUDA FOUND")
cuda_add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${CU_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} ${OpenCV_LIBS})
else ()
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${CU_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ DESTINATION include/darknet
            FILES_MATCHING PATTERN "*.h" )
 