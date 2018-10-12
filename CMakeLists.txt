cmake_minimum_required(VERSION 2.8.3)
project(rosie_object_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge std_msgs image_transport)
find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge std_msgs image_transport
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_object_detector src/motor_object_detector.cpp)
target_link_libraries(rosie_object_detector ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(rosie_object_detector sensor_msgs cv_bridge std_msgs image_transport)
