#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <cmath>
#include <Eigen/Dense> 
#include "../include/aubo_kinematics.h"
typedef pcl::PointXYZ pointxyz;
using namespace std;
typedef pcl::PointXYZRGB pointrgb;

int main (int argc, char** argv)
{
    if (argc < 2)
    {
        printf ("No target PCD file given!\n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>)
                                         ,cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          final (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_3 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_4 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_5 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_6 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_7 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_8 (new pcl::PointCloud<pcl::PointXYZRGB>);
                                          
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud_0)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    *final=*final+*cloud_0;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2],*cloud_1)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
     *final=*final+*cloud_1;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[3],*cloud_2)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    *final=*final+*cloud_2;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[4],*cloud_3)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
     *final=*final+*cloud_3;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[5],*cloud_4)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    *final=*final+*cloud_4;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[6],*cloud_5)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
     *final=*final+*cloud_5;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[7],*cloud_6)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    *final=*final+*cloud_6;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[8],*cloud_7)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
     *final=*final+*cloud_7;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[9],*cloud_8)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    *final=*final+*cloud_8;
    pcl::PCDWriter writer;
    std::stringstream sss;
    sss << "cloud_registration_final" << ".pcd";
    writer.write<pcl::PointXYZRGB> (sss.str (), *final, false); 
    
  return (0);
}