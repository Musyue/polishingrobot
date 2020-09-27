#include "polishingrobot_onlineplanner/aubo_10_polishing_opreating.h"
using aubo10_polishing_control::Aubo10Polishing;

Aubo10Polishing::Aubo10Polishing()
{
    n_private = ros::NodeHandle("aubo_10_polishing_opreating");
    // Pub_Sub_Setup();
}
void Aubo10Polishing::Pub_Sub_Setup()
{
//   feature_sub = n_private.subscribe ("smarteye_shortest_path_point_output", 1, &Aubo10Polishing::cloud_cb_callback);
  
}
void Aubo10Polishing::Print_cloud()
{
    std::cout<<sub_cloud->size()<<std::endl;
    for (size_t i = 0; i < sub_cloud->size(); i++)
    {
        std::cout<<i<<"Point->"<<" "
        <<sub_cloud->points[i].x<< " "
        <<sub_cloud->points[i].y<< " "
        <<sub_cloud->points[i].z<< " Z distance >>"
        <<sub_cloud->points[i].intensity<< " "
        <<"Normal: "
        <<sub_cloud->points[i].normal_x<< " "
        <<sub_cloud->points[i].normal_y<< " "
        <<sub_cloud->points[i].normal_z<< " "
        <<sub_cloud->points[i].curvature<< " "
        <<std::endl;
    }
    std::cout<<"========================"<<std::endl;
}
void Aubo10Polishing::cloud_cb_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*sub_cloud);
}
