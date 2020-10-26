#ifndef AUBO10_ROS_DRIVER_H
#define AUBO10_ROS_DRIVER_H
//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
//aubo
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <complex>

#include "aubo_cplusplus_control/util.h"
//yaml
#include <yaml-cpp/yaml.h>
//regx
#include <regex>
#define SERVER_HOST "192.168.1.115"
#define SERVER_PORT 8899
#define LENARRAY(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))
namespace aubo10_ros_driver {
    struct aubo_movej
    {
        double joint0;
        double joint1;
        double joint2;
        double joint3;
        double joint4;
        double joint5;
        double acc;
        double vel;
        int gozero;

    };
    class AuboRosDriver {
        public:
            AuboRosDriver();
            ~AuboRosDriver() {};
            void init_aubo_driver();
            void shutdown_aubo();
            void logout_aubo();

            //MOVEJ
            bool aubo_movej_one(double jointdeg[],double parmlist[]);
            bool aubo_movej_path(double jointdeg[][6],double parmlist[]);
            //movel
            bool aubo_movel(double jointdeg[][6],double parmlist[]);
            bool aubo_movet(double jointdeg[][6],double parmlist[]);
            //for joint
            void aubo_joint_set_acc(double joint_acc);
            void aubo_joint_set_vel(double joint_vel);
            //for end
            void aubo_end_set_acc(double line_acc,double angle_acc);//line and angle
            void aubo_end_set_vel(double line_acc,double angle_acc);//line and angle


            void MoveJ_One_Callback(const std_msgs::String::ConstPtr& msg);
            //MOVEL
            void MoveL_Callback(const std_msgs::String::ConstPtr& msg);
            //MOVET

            void MoveT_Callback(const std_msgs::String::ConstPtr& msg);


        private:
            //Use for read yaml data
            YAML::Node config;
            std::string AuboIP="";
            int AuboPort;
            ServiceInterface robotService;
            int ret;
    };
}
#endif