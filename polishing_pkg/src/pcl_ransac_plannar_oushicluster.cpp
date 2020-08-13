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

typedef pcl::PointXYZ point;
using namespace std;
typedef pcl::PointXYZRGB pointrgb;

int main (int argc, char** argv)
{
    if (argc < 2)
    {
        printf ("No target PCD file given!\n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>),cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>)
                                        ,cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        final (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr aftercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud_0)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    // printf("woqu\n");
    auto start = std::chrono::steady_clock::now();
    for(size_t i=0;i<cloud_0->points.size();++i)
    {
      // std::cout<< "num:"<<i<<"  "<< cloud_0->points[i].x<<"  "<<cloud_0->points[i].y<<" "<<cloud_0->points[i].z<<"  "<< cloud_0->points[i].rgb<<std::endl;
      pointrgb p;
      p.x=(cloud_0->points[i].x)/1000.0f;
      p.y=(cloud_0->points[i].y)/1000.0f;
      p.z=(cloud_0->points[i].z)/1000.0f;
      p.rgb=cloud_0->points[i].rgb;
      cloud->points.push_back(p);
    }
    cloud->width=1;
    cloud->height=cloud_0->points.size();
    
    // pause();
    //set filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.90, 1.100);
    pass.filter (*cloud_filtered_2);

    pcl::PassThrough<pcl::PointXYZRGB> passx;
    passx.setInputCloud (cloud_filtered_2);
    passx.setFilterFieldName ("x");
    passx.setFilterLimits (-0.43, 0.43);
    passx.filter (*cloud_filtered_3);

    pcl::PassThrough<pcl::PointXYZRGB> passy;
    passy.setInputCloud (cloud_filtered_3);
    passy.setFilterFieldName ("y");
    passy.setFilterLimits (-0.22, 0.22);
    passy.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
    sor1.setInputCloud (cloud_filtered);
    sor1.setLeafSize (0.003f, 0.003f, 0.003f);
    sor1.filter (*cloud_filtered_1);
    std::cerr << "PointCloud after VoxelGrid has: "
            << cloud_filtered_1->points.size () << " data points." << std::endl;
    pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZRGB>("table_scene_lms400_downsampled.pcd", *cloud_filtered_1, false);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //创建一个PointIndices结构体指针
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // 可选
    seg.setOptimizeCoefficients(true); //设置对估计的模型做优化处理
    // 必选
    seg.setModelType(pcl::SACMODEL_PLANE);//设置分割模型类别
    seg.setMethodType(pcl::SAC_RANSAC);//设置使用那个随机参数估计方法
    seg.setMaxIterations(1000);//迭代次数
    seg.setDistanceThreshold(0.0018);//设置是否为模型内点的距离阈值

    // 创建滤波器对象
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int i = 0, nr_points = (int)cloud_filtered_1->points.size();
    // 当还多于30%原始点云数据时
    
    while (cloud_filtered_1->points.size() > 0.3 * nr_points)
    {
        // 从余下的点云中分割最大平面组成部分
        seg.setInputCloud(cloud_filtered_1);
        seg.segment(*inliers, *coefficients);
        std::cerr << "PointCloud Ransac:ax+by+cz+d=0 "
            <<"a:"<< coefficients->values[0] << " "
            <<"b:"<< coefficients->values[1] << " "
            <<"c:"<< coefficients->values[2] << " "
            <<"d:"<< coefficients->values[3] << " "
             << std::endl;
        if (inliers->indices.size() == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }
        // 分离内层
        extract.setInputCloud(cloud_filtered_1);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        cout << "cloud_filtered_1: " << cloud_filtered_1->size() << endl;//输出提取之后剩余的

        cout << "----------------------------------" << endl;
        //保存
        cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << endl;
        // std::stringstream ss;
        // ss << "table_scene_lms400_plane_" << i << ".pcd"; //对每一次的提取都进行了文件保存
        // writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_p, false);
        // 创建滤波器对象
        extract.setNegative(true);//提取外层
        extract.filter(*cloud_f);//将外层的提取结果保存到cloud_f
        std::cerr << "PointCloud Extract Points: "
            << cloud_f->points.size () << " data points." << std::endl;

        // pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
        // sor1.setInputCloud (cloud_f);
        // sor1.setLeafSize (0.004f, 0.004f, 0.004f);
        // sor1.filter (*final);
        // std::cerr << "PointCloud after VoxelGrid has: "
        //     << final->points.size () << " data points." << std::endl;
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
        // sor1.setInputCloud(cloud_f);
        // sor1.setMeanK(50);
        // sor1.setStddevMulThresh(1);
        // sor1.filter(*final);
        // cout<<final->size()<<endl;
		    // std::stringstream sss;
        // sss << "out_" << i << ".pcd"; //对每一次的提取都进行了文件保存
        // writer.write<pcl::PointXYZRGB>(sss.str(), *final, false);
        cloud_filtered_1.swap(cloud_f);//经cloud_filtered与cloud_f交换

        i++;
    }

    cout << "cloud_filtered: " << cloud_filtered_1->size() << endl;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg1(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg2(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);

    // pcl::io::loadPCDFile("table_scene_lms400_plane_0.pcd", *cloud_seg1);
    // pcl::io::loadPCDFile("out_0.pcd", *cloud_seg2);
    // pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud_voxel);
    /*
    //将提取结果进行统计学滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
    sor1.setInputCloud(cloud_seg2);
    sor1.setMeanK(50);
    sor1.setStddevMulThresh(1);
    sor1.filter(*cloud_f);
    cout<<cloud_f->size()<<endl;
  */
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
    ne.setInputCloud (cloud_filtered_1);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 5cm
    ne.setRadiusSearch (0.1);

    // Compute the features
    ne.compute (*cloud_normals);
    // for (size_t i = 0; i < cloud_normals->size(); i++)
    // {
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::micro> elapsed = end - start;
    std::cout<< "time: "  << elapsed.count()/10000000.0f << "s" << std::endl;
    std::cout<<"cloud_normals normal size:"<<cloud_normals->size()<<" "<<std::endl;
    std::stringstream sss;
    sss << "cloud_filtered_1" << ".pcd";
    writer.write<pcl::PointXYZRGB> (sss.str (), *cloud_filtered_1, false); //*

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_extrac (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_extrac->setInputCloud (cloud_filtered_1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_last(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01); // 5cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree_extrac);
    ec.setInputCloud (cloud_filtered_1);
    ec.extract (cluster_indices);
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width =1;
      cloud_cluster->height =  cloud_cluster->points.size ();
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
      std::cout<<"Point:"<<cloud_cluster->points[cloud_cluster->size()/2].x<<" "
      <<cloud_cluster->points[cloud_cluster->size()/2].y<<" "
      <<cloud_cluster->points[cloud_cluster->size()/2].z<<" "
      <<std::endl;
      pointrgb p;
      p.x=cloud_cluster->points[cloud_cluster->size()/2].x;
      p.y=cloud_cluster->points[cloud_cluster->size()/2].y;
      p.z=cloud_cluster->points[cloud_cluster->size()/2].z;
      p.rgb=cloud_cluster->points[cloud_cluster->size()/2].rgb;
      cloud_last->points.push_back(p);
      j++;
    }
    cloud_last->width=1;
    cloud_last->height=j;

    // for (size_t i = 0; i < cloud_normals_1->size(); i++)
    // {
    //   /* code */
    //   std::cout<<"Point:"<<" "<<cloud_last->points[i].x<<" "<<cloud_last->points[i].y<<" "
    //   <<cloud_last->points[i].z<<" "
    //   <<std::endl;
    //   std::cout<<"Normal:"<<" "<<-1.0f*cloud_normals_1->points[i].normal_x<<" "<<-1.0f*cloud_normals_1->points[i].normal_y<<" "
    //   <<-1.0f*cloud_normals_1->points[i].normal_z<<" "
    //   <<std::endl;
    // }
    //   std::cout<<"NormalPointToPlaneError:"
    //   <<cloud_filtered_1->points[i].z-(-coefficients->values[3]-coefficients->values[0]*(-cloud_filtered_1->points[i].x)-coefficients->values[1]*(cloud_filtered_1->points[i].y))/coefficients->values[2]<<" "
    //   <<std::endl;
    //   // std::cout
    //   // <<cloud_filtered_1->points[i].z-(-coefficients->values[3]-coefficients->values[0]*(cloud_filtered_1->points[i].x)-coefficients->values[1]*(cloud_filtered_1->points[i].y))/coefficients->values[2]<<" "
    //   // <<std::endl;
    // }
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->initCameraParameters();
    // viewer->addCoordinateSystem (0.5);
    int v1(0);
    viewer->createViewPort(0, 0, 0.25, 1, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color1(cloud_last, 255, 0, 0);
    viewer->addPointCloud(cloud_last, color1, "cloud_last", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_last");
    // viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_last, cloud_normals_1,20,0.05,"cloud_normals_1",v1);
    // viewer->addCoordinateSystem (0.5);
    int v2(0);
    viewer->createViewPort(0.25, 0, 0.5, 1, v2);
    viewer->setBackgroundColor(0, 255, 255, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color2(cloud_p, 244, 89, 233);
    viewer->addPointCloud(cloud_p, "cloud_p", v2); 
    // viewer->addCoordinateSystem (0.5);
    int v3(0);
    viewer->createViewPort(0.5, 0, 0.75, 1, v3);
    viewer->setBackgroundColor(0,0,0, v3);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color3(cloud_filtered_1, 255, 0, 0);
    viewer->addPointCloud(cloud_filtered_1, color3,"cloud_filtered_1", v3);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered_1");
    viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_filtered_1, cloud_normals,20,0.05,"cloud_normals",v3);
    // viewer->addCoordinateSystem (0.5);
    int v4(0);
    viewer->createViewPort(0.75, 0, 1, 1, v4);
    viewer->setBackgroundColor(0, 0, 255, v4);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color4(cloud_filtered, 244, 89, 233);
    
    viewer->addPointCloud(cloud_filtered, "cloud_statical", v4);
    // viewer->addCoordinateSystem (0.5);
   // viewer->addCoordinateSystem();//添加坐标系

    viewer->spin();
    
  return (0);
}