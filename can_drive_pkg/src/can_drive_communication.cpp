#include<stdint.h>
#include<stdio.h>
#include<ros/ros.h>											// 包含ROS的头文件
#include<ros/time.h>
#include<math.h>
#include<array>
#include<geometry_msgs/TransformStamped.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<fstream>
#include<iostream>
#include"can_drive_pkg/motor_control.h"

Motor_Control motor_Ctr;

#define   L       0.498  //轮间距
#define   R       0.249
#define   r_to_m_Switch  9000.0    //(pi*d轮子直径)/(reduce_num*60)s  转每分转换成m每秒
#define   extreme_low  1038500000.0  //1038500000.0    0
#define   extreme_high  1100000000.0    //1100000000.0   -60293120 
#define   length       0.465
ros::Time current_time, last_time;

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9}};


struct robotSpeed1{
  INT16 Motor_ID1=0x202; //mode
  INT16 Motor_ID2=0x302;  //speed
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed; 
 
} left_Wheel;


struct robotSpeed2{
  INT16 Motor_ID1=0x201; //mode
  INT16 Motor_ID2=0x301; //speed
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed;
 
} right_Wheel;

struct robotSpeed3{
  INT16 Motor_ID1=0x203;
  INT16 Motor_ID2=0x303;  
  BYTE Control_Word1[2]={0x1f,0x00} ;
  BYTE Control_Word2[2]={0x2f,0x00} ;
  BYTE Work_Mode=0x01;
  int target_position;
  int Speed;
 
} up_down_motor;





double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

union odometry													//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right, temprature, odom_yaw, odom_pitch, odom_roll;

void readData()
{
  motor_Ctr.Motor_Feedback();

  ros::Time curr_time;
  vx= (motor_Ctr.left_realtime_Speed+motor_Ctr.right_realtime_Speed)*PI/(r_to_m_Switch*2);
  vth=((motor_Ctr.right_realtime_Speed-motor_Ctr.left_realtime_Speed)*PI)/(L*r_to_m_Switch);
  curr_time = ros::Time::now();

  double dt = (curr_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
  last_time = curr_time;
  //ROS_INFO("vx:%f,vth:%f",vx,vth) ;
  //ROS_INFO("x:%f,y:%f",x,y) ;
}

void writeSpeed(const geometry_msgs::Twist& msg)
{
	double RobotV = (msg.linear.x*r_to_m_Switch)/PI;
	double YawRate = msg.angular.z;
	double r = msg.linear.x / YawRate;
	if(RobotV == 0)
	{
		left_Wheel.Speed = (YawRate * R*r_to_m_Switch)/PI;
		right_Wheel.Speed = (YawRate * R*r_to_m_Switch)/PI;
	} 
  else if(YawRate == 0)
	{
		left_Wheel.Speed =-RobotV;
		right_Wheel.Speed =RobotV;
	}
	else
	{
		right_Wheel.Speed =(YawRate * (r + R)*r_to_m_Switch)/PI;
		left_Wheel.Speed = -(YawRate * (r - R)*r_to_m_Switch)/PI;
	}
  //ROS_INFO("left_wheel_Speed:%d",left_Wheel.Speed);
  //ROS_INFO("right_wheel_Speed:%d",right_Wheel.Speed);
  //motor_Ctr.Motor_Speed_Control(left_Wheel.Motor_ID, left_Wheel.Speed, left_Wheel.Work_Mode,left_Wheel.Control_Word);
	//motor_Ctr.Motor_Speed_Control(right_Wheel.Motor_ID, right_Wheel.Speed, right_Wheel.Work_Mode,right_Wheel.Control_Word);
  motor_Ctr.Motor_Speed_Control(left_Wheel.Motor_ID2, left_Wheel.Speed);
  motor_Ctr.Motor_Mode_Control(left_Wheel.Motor_ID1,left_Wheel.Work_Mode,left_Wheel.Control_Word);
	motor_Ctr.Motor_Speed_Control(right_Wheel.Motor_ID2, right_Wheel.Speed);
  motor_Ctr.Motor_Mode_Control(right_Wheel.Motor_ID1,right_Wheel.Work_Mode,right_Wheel.Control_Word);
}


void write_lift_Speed(const sensor_msgs::JointState& msg)
{
   up_down_motor.Speed=msg.velocity[0]*120000.0;
   up_down_motor.target_position=extreme_high-((msg.position[0]/length)*(extreme_high-extreme_low-1));
   motor_Ctr.Motor_Lift_Control(up_down_motor.Motor_ID2,up_down_motor.target_position,up_down_motor.Speed);
   motor_Ctr.Motor_Mode_Control(up_down_motor.Motor_ID1,up_down_motor.Work_Mode,up_down_motor.Control_Word2);
   motor_Ctr.Motor_Mode_Control(up_down_motor.Motor_ID1,up_down_motor.Work_Mode,up_down_motor.Control_Word1);
   ROS_INFO("lift_speed:%d",up_down_motor.Speed);
   ROS_INFO("lift_level:%d",up_down_motor.target_position);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_drive_communication");										//初始化节点
    ros::Time::init();
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(20);
    ros::NodeHandle nh;
    ros::NodeHandle privatenh("test_ns");
    std::ofstream out("test1.txt");
    if(out.is_open())
    {
        out << "This is a line.\n";
        out << "This is another line.\n";
        out.close();
    }
  

    // ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 50); 
    ros::Publisher pub1=nh.advertise<sensor_msgs::JointState>("odom_msg",50);
    tf::TransformBroadcaster odom_broadcaster;
    sensor_msgs::JointState odom_msg;
    ros::Subscriber sub1 = nh.subscribe("cmd_vel", 50, &writeSpeed);
    ros::Subscriber sub2 = nh.subscribe("lift_msg", 50,&write_lift_Speed);
    geometry_msgs::TransformStamped odom_trans;
    motor_Ctr.Motor_PDO_Open();
    // motor_Ctr.Motor_Mode_Control(left_Wheel.Motor_ID1,left_Wheel.Work_Mode,left_Wheel.Control_Word);
    //motor_Ctr.Motor_Mode_Control(right_Wheel.Motor_ID1,right_Wheel.Work_Mode,right_Wheel.Control_Word);
    motor_Ctr.Motor_Speed_Control(left_Wheel.Motor_ID2,0);
    motor_Ctr.Motor_Mode_Control(left_Wheel.Motor_ID1,left_Wheel.Work_Mode,left_Wheel.Control_Word);
    motor_Ctr.Motor_Speed_Control(right_Wheel.Motor_ID2,0);
    motor_Ctr.Motor_Mode_Control(right_Wheel.Motor_ID1,right_Wheel.Work_Mode,right_Wheel.Control_Word);
    ROS_INFO("ROS Node initialized successful.");
    nav_msgs::Odometry msgl;

    
    while(ros::ok()) 
    {
        motor_Ctr.Motor_Feedback();
        //readData();
        current_time = ros::Time::now();
        odom_msg.header.stamp=current_time;
        odom_msg.name.resize(4);
        odom_msg.position.resize(4);
        odom_msg.velocity.resize(4);
        odom_msg.name[0]="left_dis";
        odom_msg.position[0]=motor_Ctr.left_dis;
        odom_msg.velocity[0]=motor_Ctr.left_realtime_Speed;
        odom_msg.name[1]="right_dis";
        odom_msg.position[1]=motor_Ctr.right_dis;
        odom_msg.velocity[1]=motor_Ctr.right_realtime_Speed;
        odom_msg.name[2]="left_dis_r";
        odom_msg.position[2]=motor_Ctr.left_dis_value;
        odom_msg.name[3]="rigth_dis_r";
        odom_msg.position[3]=motor_Ctr.right_dis_value;
        pub1.publish(odom_msg);


        current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);        /*角度转换成四元数坐标*/    
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
                       
   

        //send the transform
        //odom_broadcaster.sendTransform(odom_trans);


        msgl.header.stamp = current_time;
        msgl.header.frame_id = "odom";
        /*设置位置*/
        msgl.pose.pose.position.x = x;
        msgl.pose.pose.position.y = y;
        msgl.pose.pose.position.z = 0.0;
        msgl.pose.pose.orientation = odom_quat;
        msgl.pose.covariance=odom_pose_covariance;
        /*设置方向*/
        msgl.child_frame_id = "base_link";
        msgl.twist.twist.linear.x = vx;
        msgl.twist.twist.linear.y = vy;
        msgl.twist.twist.angular.z = vth;
        msgl.twist.covariance=odom_twist_covariance;
   
        //pub.publish(msgl);   
        ros::spinOnce();
        loop_rate.sleep();
    }
  
  
    return 0;
    
}
