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
geometry_msgs::Pose global_msg_tmp;
bool data_recv=false;
geometry_msgs::Pose start_pose_global_msg_tmp;
bool start_data_recv=false;
void binpacking_PathCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  global_msg_tmp.position.x=msg->position.x;
  global_msg_tmp.position.y=msg->position.y;
  global_msg_tmp.position.z=msg->position.z;
  global_msg_tmp.orientation.x=msg->orientation.x;
  global_msg_tmp.orientation.y=msg->orientation.y;
  global_msg_tmp.orientation.z=msg->orientation.z;
  global_msg_tmp.orientation.w=msg->orientation.w;
  ROS_INFO("I heard: [%f]", msg->position.x);//msg->data.c_str());
  data_recv=true;
}
void binpacking_start_PathCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  start_pose_global_msg_tmp.position.x=msg->position.x;
  start_pose_global_msg_tmp.position.y=msg->position.y;
  start_pose_global_msg_tmp.position.z=msg->position.z;
  start_pose_global_msg_tmp.orientation.x=msg->orientation.x;
  start_pose_global_msg_tmp.orientation.y=msg->orientation.y;
  start_pose_global_msg_tmp.orientation.z=msg->orientation.z;
  start_pose_global_msg_tmp.orientation.w=msg->orientation.w;
  ROS_INFO("Start Point I heard: [%f]", msg->position.x);//msg->data.c_str());
  start_data_recv=true;

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_aubo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  static const std::string PLANNING_GROUP = "aubo10";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  ros::Subscriber sub = node_handle.subscribe("move_path_from_bin_packing", 10, binpacking_PathCallback);
  ros::Subscriber start_sub = node_handle.subscribe("move_path_from_bin_packing_start_pose", 10, binpacking_start_PathCallback);
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
    if(data_recv)
    {
      ROS_INFO("Upate data Pose: x: [%f] y: [%f] z: [%f] Data Ori: x: [%f] y: [%f] z: [%f] w: [%f]", 
              global_msg_tmp.position.x,
              global_msg_tmp.position.y,
              global_msg_tmp.position.z,
              global_msg_tmp.orientation.x,
              global_msg_tmp.orientation.y,
              global_msg_tmp.orientation.z,
              global_msg_tmp.orientation.w);
      robot_state::RobotState start_state(*move_group.getCurrentState());
      // if(start_data_recv)
      // {//0.5006904062010249, 0.52573641767206825, 0.49718886757243203, 0.47509321846731939
        geometry_msgs::Pose start_pose2;
        start_pose2.orientation.x = 0.5006904062010249;//start_pose_global_msg_tmp.orientation.x;
        start_pose2.orientation.y = 0.52573641767206825;//start_pose_global_msg_tmp.orientation.y;
        start_pose2.orientation.z = 0.49718886757243203;//start_pose_global_msg_tmp.orientation.z;
        start_pose2.orientation.w = 0.47509321846731939;//start_pose_global_msg_tmp.orientation.w;
        start_pose2.position.x    = 0.41382214336418305;//start_pose_global_msg_tmp.position.x;
        start_pose2.position.y    = -0.16260875941470343;//start_pose_global_msg_tmp.position.y;
        start_pose2.position.z    = 0.54260208096811613;//start_pose_global_msg_tmp.position.z;
        start_state.setFromIK(joint_model_group, start_pose2);

        move_group.setStartState(start_state);
      // }else
      // {
      //   ROS_INFO("wait get the start pose ---");
      // }
      

      geometry_msgs::Pose target_pose1;

      target_pose1.orientation.x = global_msg_tmp.orientation.x;
      target_pose1.orientation.y = global_msg_tmp.orientation.y;
      target_pose1.orientation.z = global_msg_tmp.orientation.z;
      target_pose1.orientation.w = global_msg_tmp.orientation.w;
      target_pose1.position.x = global_msg_tmp.position.x;
      target_pose1.position.y = global_msg_tmp.position.y;
      target_pose1.position.z = global_msg_tmp.position.z;
      move_group.setPoseTarget(target_pose1);
      // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(success)
      {
        ROS_ERROR("We got the path planning ----");
        data_recv=false;
        start_data_recv=false;
        if(ros::param::has("/planning_over_flag"))
        {
          ros::param::set("/planning_over_flag",1);
        }
      }
      ROS_INFO_NAMED("move_group_aubo", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    }else
    {
      ROS_INFO("Wait the target point---");
    }
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
