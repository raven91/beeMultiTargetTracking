cmake_minimum_required(VERSION 3.10)
project(beeMultiTargetTracking)

set(CMAKE_CXX_STANDARD 17)

add_executable(beeMultiTargetTracking main.cpp Parameters/ParameterHandler.cpp Parameters/ParameterHandler.hpp Definitions.hpp ImageProcessing/ImageProcessingEngine.cpp ImageProcessing/ImageProcessingEngine.hpp Tracking/MultiTargetTracker.cpp Tracking/MultiTargetTracker.hpp Tracking/HungarianAlgorithm.cpp Tracking/HungarianAlgorithm.hpp Tracking/KalmanFilter.cpp Tracking/KalmanFilter.hpp)

include_directories(/usr/local/include/opencv4/)
link_directories(/usr/local/lib/)

file(GLOB LIBRARIES "/usr/local/lib/libopencv_core.3.1.0.dylib"
        "/usr/local/lib/libopencv_highgui.3.1.0.dylib"
        "/usr/local/lib/libopencv_imgcodecs.3.1.0.dylib"
        "/usr/local/lib/libopencv_imgproc.3.1.0.dylib")
message("LIBRARIES = ${LIBRARIES}")

target_link_libraries(beeMultiTargetTracking ${LIBRARIES})

#set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS} -stdlib=libstdc++”)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
CHECK_CXX_COMPILER_FLAG("-O3" COMPILER_SUPPORTS_O3)
if(COMPILER_SUPPORTS_O3)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()
CHECK_CXX_COMPILER_FLAG("-flto" COMPILER_SUPPORTS_FLTO)
if(COMPILER_SUPPORTS_FLTO)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
endif()