#include "polishingrobot_onlineplanner/aubo_10_polishing_opreating.h"

using aubo10_polishing_control::Aubo10Polishing; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "aubo_10_polishing_opreating_node");
  ros::NodeHandle n;
  
  int aubo_10_ros_pub_hz;
  if (!ros::param::get("aubo_10_ros_pub_hz", aubo_10_ros_pub_hz)) {
    aubo_10_ros_pub_hz = 10;
  }
  Aubo10Polishing aubo10polishing;
  ros::Subscriber feature_sub = n.subscribe ("smarteye_shortest_path_point_output", 1, &Aubo10Polishing::cloud_cb_callback,&aubo10polishing);
//   ImuReader imu_reader;
  ros::Rate r(aubo_10_ros_pub_hz);
  while (ros::ok()) {
    // imu_reader.ReadData();
    aubo10polishing.Print_cloud();
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}