#include "aubo_cplusplus_control/aubo_ros_driver.h"

using aubo10_ros_driver::AuboRosDriver; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "aubo_ros_driver");
  ros::NodeHandle n;
  
  int aubo_10_ros_pub_hz;
  if (!ros::param::get("aubo_10_ros_pub_hz", aubo_10_ros_pub_hz)) {
    aubo_10_ros_pub_hz = 10;
  }
  AuboRosDriver a1rd;
  a1rd.init_aubo_driver();
  ros::Subscriber movej_sub = n.subscribe ("/aubo_ros_driver/movej", 1, &AuboRosDriver::MoveJ_One_Callback,&a1rd);
  ros::Subscriber movej_path_sub = n.subscribe ("/aubo_ros_driver/movejp", 1, &AuboRosDriver::MoveJ_Path_Callback,&a1rd);
  ros::Subscriber movel_path_sub = n.subscribe ("/aubo_ros_driver/movel", 1, &AuboRosDriver::MoveL_Callback,&a1rd);
  ros::Subscriber movet_path_sub = n.subscribe ("/aubo_ros_driver/movet", 1, &AuboRosDriver::MoveT_Callback,&a1rd);
  ros::Rate r(aubo_10_ros_pub_hz);
  ROS_INFO("====Aubo ROS Driver is OK====");
  while (ros::ok()) {
    ROS_INFO("Waiting your cmd please publish the [movej,movejp,movet,movel]");
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}