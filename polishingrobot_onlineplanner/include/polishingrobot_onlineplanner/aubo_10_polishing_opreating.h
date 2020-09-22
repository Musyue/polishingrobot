#ifndef AUBO_10_POLISHING_OPREATING_H
#define AUBO_10_POLISHING_OPREATING_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
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

namespace aubo10_polishing_control {
    class Aubo10Polishing {
        public:
            Aubo10Polishing();
            ~Aubo10Polishing() {};
            void Pub_Sub_Setup();
            void cloud_cb_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);

        private:
            ros::NodeHandle n_private;
            ros::Subscriber feature_sub;


    };
}
#endif // AUBO_10_POLISHING_OPREATING_H