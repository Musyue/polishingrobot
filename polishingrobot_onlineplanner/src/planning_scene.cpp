#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


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
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 1;

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

    ros::init(argc, argv, "planning_scene_ros_api_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle;

    // modified by yzf 
   
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));

    // const robot_model::RobotModelPtr& kinematic_model = robot_model_loader->getModel();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;


  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  if(planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startSceneMonitor("/planning_scene");
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();
  }
  else
  {
    ROS_ERROR("Error in setting up the PlanningSceneMonitor.");

    exit(EXIT_FAILURE);
  }

    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();
    robot_state::RobotState& 
    current_state = planning_scene->getCurrentStateNonConst();

    /* add a single object to ps's world */
    add_objects(planning_scene_interface);

    sleep(2);


    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {

        bool exist_dae_mesh = planning_scene->getWorld()->hasObject("dae_mesh");
        ROS_WARN("exist_dae_mesh: %d", exist_dae_mesh);

        // current_state.update();
        ROS_WARN("printStatePositions:");
        current_state.printStatePositions();

        collision_result.clear();
        
        planning_scene->checkSelfCollision(collision_request, collision_result, current_state);

        ROS_INFO_STREAM("Test SelfCollision Check  : Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
        std::cout<<"collision_result.collision "<< collision_result.collision<<std::endl;
        if(ros::param::has("/judge_self_collision_flag"))
        {
          ros::param::set("/judge_self_collision_flag",collision_result.collision);
        }
        collision_result.clear();
        collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
        planning_scene->checkCollision(collision_request, collision_result, current_state, acm);
        ROS_INFO_STREAM("Test Collision With Environment Check  : Current state is " << (collision_result.collision ? "in" : "not in") << " collision with environment");
        // ros::param::set("/judge_self_collision_flag",0);


        loop_rate.sleep();
    }
  

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
