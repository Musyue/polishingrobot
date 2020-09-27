cmake_minimum_required(VERSION 2.8.3)
project(polishingrobot_onlineplanner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PCL REQUIRED)

find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  DEPENDS
  EIGEN3
)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)



add_executable(polishingrobot_onlineplanner
  src/planning_scene.cpp
)
target_link_libraries(polishingrobot_onlineplanner ${catkin_LIBRARIES})

# add_executable(aubo10_collision_detecting
#   src/aubo10_collision_detecting.cpp
#   src/aubo_kinematics.cpp
# )
# target_link_libraries(aubo10_collision_detecting ${catkin_LIBRARIES})

add_executable(motion_planning_aubo src/motion_planning_aubo.cpp)
target_link_libraries(motion_planning_aubo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aubo_10_test_node src/aubo_10_test_node.cpp)
target_link_libraries(aubo_10_test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(aubo_10_polishing_opreating_node src/aubo_10_polishing_opreating_node.cpp src/aubo_10_polishing_opreating.cpp )
target_link_libraries(aubo_10_polishing_opreating_node yaml-cpp pthread ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} )


add_executable(marker_test src/marker_test.cpp)
target_link_libraries(marker_test ${catkin_LIBRARIES} )

# add_executable(move_group_aubo src/move_group_aubo.cpp)
# target_link_libraries(move_group_aubo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_group_aubo_tutorial src/move_group_aubo_tutorial.cpp)
target_link_libraries(move_group_aubo_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############




