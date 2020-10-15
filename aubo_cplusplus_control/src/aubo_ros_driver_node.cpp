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
  ros::Subscriber movej_sub = n.subscribe ("/aubo_ros_driver/movej", 1, &AuboRosDriver::MoveJ_Callback,&a1rd);
  ros::Subscriber movel_sub = n.subscribe ("/aubo_ros_driver/movel", 1, &AuboRosDriver::MoveL_Callback,&a1rd);
  ros::Rate r(aubo_10_ros_pub_hz);
  while (ros::ok()) {
    ROS_INFO("Waiting your cmd please publish the [movej,movet,movel]");
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}