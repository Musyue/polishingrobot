#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <sstream>
void add_objects( moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  Eigen::Vector3d world_size;
  world_size << 1,1,1;
  shapes::Mesh* load_mesh = shapes::createMeshFromResource("package://polishingrobot_onlineplanner/mesh/20200716_scan_planning_1.stl", world_size);  
  if(load_mesh==NULL) 
  {
      ROS_WARN("mesh is NULL !!!");
      return;
  } 
  
  //
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(load_mesh, mesh_msg);
  shape_msgs::Mesh mesh;
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  //定义物体方位
  geometry_msgs::Pose pose;
  pose.orientation.w =1.0;
  pose.position.x = 1.2;
  pose.position.y = -1.3;
  pose.position.z = 26.37;

  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = "map";
  obj.id="dae_mesh";
  obj.mesh_poses.push_back(pose);
  obj.meshes.push_back(mesh);
  //定义操作为添加
  obj.operation = obj.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(obj);
  planning_scene_interface.addCollisionObjects(collision_objects);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_aubo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher aubo_move_pub = node_handle.advertise<std_msgs::String>("/aubo_ros_script/movej", 1);
  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  static const std::string PLANNING_GROUP = "aubo10";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ros::Rate loop_rate(10.0);
  int flag=0;

  while (ros::ok())
  {

    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
