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
//boost
#include <boost/spirit/include/qi.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/fusion/tuple.hpp>
#define SERVER_HOST "192.168.1.115"
#define SERVER_PORT 8899
namespace aubo10_ros_driver {
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
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
    class Aubo10RosDriver {
        public:
            Aubo10RosDriver();
            ~Aubo10RosDriver() {};

            
            void init_aubo_driver();
            void shutdown_aubo();
            void logout_aubo();
            bool aubo_movej(double jointdeg[],float acc,float vel,bool gozero);
            void MoveJ_Callback(const std_msgs::String::ConstPtr& msg);

        private:
            //Use for read yaml data
            YAML::Node config;
            std::string AuboIP="";
            int AuboPort;
            ServiceInterface robotService;
            int ret;
            

    };
}
BOOST_FUSION_ADAPT_STRUCT(
    aubo10_ros_driver::aubo_movej,
    (double,joint0)
    (double, joint1)
    (double ,joint2)
    (double ,joint3)
    (double ,joint4)
    (double ,joint5)
    (double ,acc)
    (double ,vel)
    (int, gozero)
)

namespace aubo10_ros_driver
{
    template <typename Iterator>
    struct aubo_movej_parser : qi::grammar<Iterator, aubo_movej(), ascii::space_type>
    {
        aubo_movej_parser() : aubo_movej_parser::base_type(start)
        {
            using qi::int_;
            using qi::lit;
            using qi::double_;
            using qi::lexeme;
            using ascii::char_;

            start %=
                lit("movej")
                >> '{'
                >>lit("joint")
                >>'='
                >>'('
                >>  double_ >> ','
                >>  double_ >> ','
                >>  double_>> ','
                >>  double_ >> ','
                >>  double_ >> ','
                >>  double_
                >>')'
                >>','
                >>lit("acc")
                >>'='
                >>  double_>> ','
                >>lit("vel")
                >>'='
                >>  double_>> ','
                >>lit("gozero")
                >>'='
                >>  int_
                >>  '}'
                ;
        }

        //qi::rule<Iterator, std::string(), ascii::space_type> quoted_string;
        qi::rule<Iterator, aubo_movej(), ascii::space_type> start;
    };


}
#endif