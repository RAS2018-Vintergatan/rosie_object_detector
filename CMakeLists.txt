cmake_minimum_required(VERSION 2.8.3)
project(rosie_object_detector)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge std_msgs image_transport pcl_conversions tf)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  DEPENDS Boost PCL
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge std_msgs image_transport  pcl_conversions tf
  LIBRARIES 
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(rosie_object_detector src/motor_object_detector.cpp)
target_link_libraries(rosie_object_detector ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  ${PCL_LIBRARIES})
add_dependencies(rosie_object_detector sensor_msgs cv_bridge std_msgs image_transport pcl_conversions tf)
