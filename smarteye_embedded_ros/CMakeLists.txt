
cmake_minimum_required(VERSION 2.8.3)
project(smarteye_embedded_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smarteye_embedded_ros
#  CATKIN_DEPENDS pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


set(SmarteyeAPI_INCLUDE "/data/ros/yue_ws_test/src/polishingrobot/smarteye_embedded_ros/smart_eye_sdk/include")
set(SmarteyeAPI_LIBS_DIR "/data/ros/yue_ws_test/src/polishingrobot/smarteye_embedded_ros/smart_eye_sdk/lib")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SmarteyeAPI_INCLUDE}
)
link_directories(${SmarteyeAPI_LIBS_DIR})

add_executable(smarteye_with_viewpoint src/smarteye_with_viewpoint.cpp src/aubo_kinematics.cpp)
target_link_libraries(smarteye_with_viewpoint yaml-cpp emSmarteyeAPI pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(smarteye_ros_demo src/smarteye_ros_demo.cpp)
target_link_libraries(smarteye_ros_demo emSmarteyeAPI pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(smarteye_ros_detection_node src/smarteye_ros_detection_node.cpp)
target_link_libraries(smarteye_ros_detection_node emSmarteyeAPI pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(smarteye_demo src/smarteye_demo.cpp)
target_link_libraries(smarteye_demo emSmarteyeAPI pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(aubo_kinematics_test src/aubo_kinematics_test.cpp src/aubo_kinematics.cpp)
# add_dependencies(aubo_kinematics_test )
target_link_libraries(aubo_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})