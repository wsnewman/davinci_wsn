cmake_minimum_required(VERSION 2.8.3)
project(camera_syncer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
   find_package(OpenCV REQUIRED)
   include_directories(${OpenCV_INCLUDE_DIRS})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example_opencv
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
    
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cameras_timesync src/cameras_timesync.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(example_opencv_node example_opencv_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cameras_timesync ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

