#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  
#include <sensor_msgs/Joy.h> 
#include <sensor_msgs/JointState.h> 
#include <iostream>  
#include <ros/time.h>
#include<std_msgs/Bool.h>

using namespace std;  
 
class Teleop  
{  
 public:  
    Teleop();  
    double v1 = 0.2,w1 =0.2 ;
    double vmax=0.5,vmin=0.1,ang_max=0.6,ang_min=0.1;
    double v2 = 0.01,s1=0.465;
    double v2max=0.015,v2min=0.001,s1max=0.465,s1min=0;
    ros::Time current_time;
//private:  
    /* data */  
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);  
    ros::NodeHandle n; //实例化节点  
    ros::Subscriber sub ;  
    ros::Publisher pub1,pub2,pub3 ; 
    sensor_msgs::JointState lift_msg; 
    geometry_msgs::Twist v;
    std_msgs::Bool init_flag;
    double vlinear,vangular,vlift;//我们控制乌龟的速度，是通过这两个变量调整  
    int axis_ang,axis_lin,axis_lift,ton0,ton1, ton7,ton6,ton2,ton3,ton4,ton5;  //axes[]的键  
 
};  
  
Teleop::Teleop()  
{     

    n.param<int>("axis_linear",axis_lin,1); //默认axes[1]接收速度   
    n.param<int>("axis_angular",axis_ang,0);//默认axes[2]接收角度
    n.param<int>("axis_lift",axis_lift,3);
    n.param<double>("vel_linear",vlinear,v1);//默认线速度1 m/s
    n.param<double>("vel_angular",vangular,w1);//默认角速度1 单位rad/s
    n.param<double>("vel_lift",vlift,v2);
    n.param<int>("button",ton4,4);
    n.param<int>("button",ton5,5);
    n.param<int>("button",ton0,0);
    n.param<int>("button",ton1,1);
    n.param<int>("button",ton2,2);
    n.param<int>("button",ton3,3);
    n.param<int>("button",ton6,7);
    n.param<int>("button",ton7,8);
    pub1 = n.advertise<geometry_msgs::Twist>("cmd_vel",1);//将速度发给乌龟
    pub2 = n.advertise<sensor_msgs::JointState>("lift_msg",1);
    pub3 = n.advertise<std_msgs::Bool>("init_system",1);//initialization 
    sub = n.subscribe<sensor_msgs::Joy>("joy",10,&Teleop::callback,this); //订阅游戏手柄发来的数据
    //ROS_INFO("%f,%f",vlinear,vangular);
}
void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)  
{  
   if(Joy->buttons[ton7])
   {
        //ros::param::set("init_flag" ,true);
	init_flag.data = true; 
    }
   else
   {
       init_flag.data = false;
   }
   pub3.publish(init_flag); 
   if(Joy->buttons[ton4])
   {
     if(Joy->buttons[ton0])
     {
         if(vlinear<vmax)
         {
            vlinear=vlinear+0.01;
         }
     }
     else if(Joy->buttons[ton2])
     {
         if(vlinear>vmin)
         {
             vlinear=vlinear-0.01;
         }
     }
     else if(Joy->buttons[ton3])
     {
         if(vangular<ang_max)
         {
             vangular=vangular+0.01;
         }
     }
     else if(Joy->buttons[ton1])
     {
         if(vangular>ang_min)
         {
             vangular=vangular-0.01;
         }
     }
     v.linear.x =Joy->axes[axis_lin]*vlinear; //将游戏手柄的数据乘以你想要的速度，然后发给小车
     v.angular.z =Joy->axes[axis_ang]*vangular;  
     if(v.linear.x<0)
     {
         v.angular.z=-v.angular.z;
     }

     ROS_INFO("linear:%.3lf angular:%.3lf",v.linear.x,v.angular.z);   
     //ROS_INFO("v_scale:%f ang_scale:%f",vlinear,vangular);
     pub1.publish(v);  
    }
    if(Joy->buttons[5])//Joy->buttons[5]
    {   
    
      if(Joy->buttons[ton0])
      {
          if(vlift<v2max)
          {
              vlift=vlift+0.001;
          }
      }
       else if(Joy->buttons[ton2])
      {
          if(vlift>v2min)
          {
              vlift=vlift-0.001;
          }
      }
      else if(Joy->buttons[ton3])
      {
          if(s1<s1max)
          {
              s1=s1+0.005;
          }
      }
      else if(Joy->buttons[ton1])
      {
          if(s1>s1min)
          {
              s1=s1-0.005;
          }
      }
      
      current_time = ros::Time::now();
      lift_msg.header.stamp=current_time;
      lift_msg.name.resize(1);
      lift_msg.position.resize(1);
      lift_msg.velocity.resize(1);
      lift_msg.name[0]="lift_dis";
      //增加param
      //ros::param::get("js_turtle/lift_position",s1);
      //ros::param::get("js_turtle/lift_speed",vlift);
      lift_msg.position[0]=s1;
      lift_msg.velocity[0]=vlift;
      
      ROS_INFO("lift_speed:%.3lf lift_level:%.3lf",vlift,s1); 
      pub2.publish(lift_msg);
    }/*
    else
    {
      current_time = ros::Time::now();
      lift_msg.header.stamp=current_time;
      lift_msg.name.resize(1);
      lift_msg.position.resize(1);
      lift_msg.velocity.resize(1);
      lift_msg.name[0]="lift_dis";
      //增加param
      ros::param::get("js_turtle/lift_position",s1);
      ros::param::get("js_turtle/lift_speed",vlift);
      lift_msg.position[0]=s1;
      lift_msg.velocity[0]=vlift; 
      ROS_INFO("lift_speed:%.3lf lift_level:%.3lf",vlift,s1); 
      pub2.publish(lift_msg);
    }*/
}  
int main(int argc,char** argv)  
{  
 ros::init(argc, argv, "logteleop");  
 Teleop telelog;
 ros::spin();
 return 0;  
}  

