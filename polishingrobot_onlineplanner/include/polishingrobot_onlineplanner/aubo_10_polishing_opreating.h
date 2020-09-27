#ifndef AUBO_10_POLISHING_OPREATING_H
#define AUBO_10_POLISHING_OPREATING_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//yaml
#include <yaml-cpp/yaml.h>



namespace aubo10_polishing_control {
    class Aubo10Polishing {
        public:
            Aubo10Polishing();
            ~Aubo10Polishing() {};
            void Pub_Sub_Setup();
            void cloud_cb_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
            void Print_cloud();
            
            static void init_aubo_driver();
        private:
            //Use for read yaml data
            YAML::Node config;
            std::vector<float> tbc_16;
            std::vector<float> CalibrationPoint;
            std::vector<float> ViewPointLowestPoint;
            std::vector<float> ViewPointFrontStartPoint;
            ros::NodeHandle n_private;
            ros::Subscriber feature_sub;
            //for reading pcl
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr sub_cloud{new pcl::PointCloud<pcl::PointXYZINormal>};

            

    };
}
#endif // AUBO_10_POLISHING_OPREATING_H