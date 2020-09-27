#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>



void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
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

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

 
  ros::Subscriber sub = nh.subscribe ("smarteye_shortest_path_point_output", 1, cloud_cb);
  

  // Spin
  ros::spin ();
}