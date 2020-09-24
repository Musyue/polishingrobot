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
#include "aubo_cplusplus_control/util.h"
//yaml
#include <yaml-cpp/yaml.h>
//boost
#include <boost/spirit/include/qi.hpp>

#define SERVER_HOST "192.168.1.115"
#define SERVER_PORT 8899
namespace aubo10_ros_driver {
    class Aubo10RosDriver {
        public:
            Aubo10RosDriver();
            ~Aubo10RosDriver() {};

            
            void init_aubo_driver();
            void shutdown_aubo();
            void logout_aubo();
            bool aubo_movej(double jointdeg[],float acc,float vel,bool gozero);
            void MoveJ_Callback(const std_msgs::String::ConstPtr& msg);
            bool parse_line(std::string const& line, std::string& name, double& a, double& b, double& c);
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