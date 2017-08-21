cmake_minimum_required(VERSION 2.8.3)
project(iiwa_kinect_calibration)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport)

find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

include_directories(
include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
include_directories(${VISP_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(iiwa_kinect_calibration src/calibrator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(iiwa_kinect_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(iiwa_kinect_calibration ${VISP_LIBRARIES})

target_link_libraries(iiwa_kinect_calibration lapack blas)
