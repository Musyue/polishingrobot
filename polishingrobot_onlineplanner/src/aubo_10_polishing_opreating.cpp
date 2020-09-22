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
void Aubo10Polishing::cloud_cb_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    std::cout<<temp_cloud->size()<<std::endl;
    for (size_t i = 0; i < temp_cloud->size(); i++)
    {
        std::cout<<i<<"Point->"<<" "
        <<temp_cloud->points[i].x<< " "
        <<temp_cloud->points[i].y<< " "
        <<temp_cloud->points[i].z<< " Z distance >>"
        <<temp_cloud->points[i].intensity<< " "
        <<"Normal: "
        <<temp_cloud->points[i].normal_x<< " "
        <<temp_cloud->points[i].normal_y<< " "
        <<temp_cloud->points[i].normal_z<< " "
        <<temp_cloud->points[i].curvature<< " "
        <<std::endl;
    }
    std::cout<<"========================"<<std::endl;
}
