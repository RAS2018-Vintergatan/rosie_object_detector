cmake_minimum_required(VERSION 2.8.3)
project(rosie_object_detector)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs cv_bridge nav_msgs std_msgs geometry_msgs image_transport pcl_conversions tf genmsg message_generation)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_service_files(FILES ObjectClassify.srv)

add_message_files(FILES RAS_Evidence.msg)

generate_messages(DEPENDENCIES std_msgs nav_msgs sensor_msgs geometry_msgs)

catkin_package(
  DEPENDS Boost PCL
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs cv_bridge std_msgs geometry_msgs image_transport  pcl_conversions tf message_runtime
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
add_dependencies(rosie_object_detector ${catkin_EXPORTED_TARGETS})
