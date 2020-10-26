#include "smarteye_embedded_ros/smarteye_with_viewpoint_and_env_explorition.h"

using smarteye_viewpoint_and_env_explorition::SmarteyeViewpointEnvExp; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "smarteye_with_viewpoint_and_env_explorition_node");
  ros::NodeHandle nh;
  
  int smarteye_10_ros_pub_hz;
  if (!ros::param::get("smarteye_10_ros_pub_hz", smarteye_10_ros_pub_hz)) {
    smarteye_10_ros_pub_hz = 10;
  }
  SmarteyeViewpointEnvExp svpe;

  svpe.registerNodeHandle(nh);
  svpe.registerPubSub();
//   ros::Subscriber movej_sub = n.subscribe ("/aubo_ros_driver/movej", 1, &SmarteyeViewpointEnvExp::MoveJ_One_Callback,&a1rd);

  ros::Rate r(smarteye_10_ros_pub_hz);
  
  while (ros::ok()) {
    ROS_INFO("Waiting your cmd please publish the [movej,movejp,movet,movel]");
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}