#include "aubo_cplusplus_control/aubo_ros_driver.h"

using aubo10_ros_driver::Aubo10RosDriver; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "aubo_ros_driver");
  ros::NodeHandle n;
  
  int aubo_10_ros_pub_hz;
  if (!ros::param::get("aubo_10_ros_pub_hz", aubo_10_ros_pub_hz)) {
    aubo_10_ros_pub_hz = 10;
  }
  Aubo10RosDriver a1rd;
  ros::Subscriber feature_sub = n.subscribe ("/aubo_ros_driver/movej", 1, &Aubo10RosDriver::MoveJ_Callback,&a1rd);
//   ImuReader imu_reader;
  ros::Rate r(aubo_10_ros_pub_hz);
  while (ros::ok()) {
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}