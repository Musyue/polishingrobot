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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>),cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>)
                                        ,cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        final (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr aftercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }

    // printf("woqu\n");
    auto start = std::chrono::steady_clock::now();
    // for(size_t i=0;i<cloud_0->points.size();++i)
    // {
    //   // std::cout<< "num:"<<i<<"  "<< cloud_0->points[i].x<<"  "<<cloud_0->points[i].y<<" "<<cloud_0->points[i].z<<"  "<< cloud_0->points[i].rgb<<std::endl;
    //   pointrgb p;
    //   p.x=(cloud_0->points[i].x)/1000.0f;
    //   p.y=(cloud_0->points[i].y)/1000.0f;
    //   p.z=(cloud_0->points[i].z)/1000.0f;
    //   p.rgb=cloud_0->points[i].rgb;
    //   cloud->points.push_back(p);
    // }
    // cloud->width=1;
    // cloud->height=cloud_0->points.size();
    
    // pause();
    //set filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.50, 1.100);
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

    Eigen::MatrixXd Matrix_eTc(4,4);
    Matrix_eTc<<-0.99960561, -0.01773356,  0.0215622 , -0.09214288,
         0.01817358, -0.99962585,  0.0204784 ,  0.21627011,
         0.0211837 ,  0.02085993,  0.99956265, -0.01512569,
         0.        ,  0.        ,  0.        ,  1.  ; 
    Eigen::MatrixXd Matrix_bTc(4,4);
    Matrix_bTc<<0.06787 ,-0.03428, 0.99710 ,0.24361234,
        -0.99755 ,0.01493, 0.06842, -0.25666829,
        -0.01723 ,-0.99930, -0.03318, 0.75667707
        ,0,0,0,1; 
    Eigen::MatrixXd Matrix_pcl_one_point(4,1);
    Eigen::MatrixXd Matrix_pcl_new(4,1);
    ///////////////////////////////////


    VectorXd q_left_1(6);
    q_left_1<<102.668,1.7223,142.53,139.6822,-9.2608,-0.687;
    VectorXd q_left_2(6);
    VectorXd q_left_3(6);
    VectorXd q_left_4(6);
    VectorXd q_left_5(6);
    q_left_2<<93.004,-31.334,156.41,97.04,1.47,91.056;
    q_left_3<<96.16,-33.616,163.94,95.046,-2.4067,-78.19622;
    q_left_4<<90.00,-77.0939,135.78,107.825,-2.406,-78.195;
    q_left_5<<85.611,-106.7425,104.4639,105.769,-2.400,-73.97553;
    VectorXd q_right_0(6);
    VectorXd q_right_1(6);
    VectorXd q_right_2(6);
    VectorXd q_right_3(6);
    VectorXd q_right_4(6);
    q_right_0<<23.33,59.92,153.939,90.712,69.41,1.07;
    q_right_1<<90.79,82.13,146.1558,133.0358,-0.3238,110.739;
    q_right_2<<96.1967,7.31,-144.93,45.29,-1.429,-18.664;
    q_right_3<<93.782,68.15,-130.93,26.358,-5.0317,-47.545;
    q_right_4<<103.392,114.308,-81.4896,133.593,-7.04969,-151.7972;

    VectorXd q_result_aubo(6);
    // if(atoi(argv[2])==1)
    std::cout<<atoi(argv[2])<<std::endl;
    switch (atoi(argv[2]))
    {
      case 1:
        /* code */
        deg_to_rad(q_result_aubo,q_left_1);
        break;
      case 2:
        /* code */
        deg_to_rad(q_result_aubo,q_left_2);
        break;
      case 3:
        /* code */
        deg_to_rad(q_result_aubo,q_left_3);
        break;
      case 4:
        /* code */
        deg_to_rad(q_result_aubo,q_left_4);
        break;
      case 5:
        /* code */
        deg_to_rad(q_result_aubo,q_left_5);
        break;
      case 6:
        /* code */
        deg_to_rad(q_result_aubo,q_right_1);
        break;
      case 7:
        /* code */
        deg_to_rad(q_result_aubo,q_right_2);
        break;
      case 8:
        /* code */
        deg_to_rad(q_result_aubo,q_right_3);
        break;
      case 9:
        /* code */
        deg_to_rad(q_result_aubo,q_right_4);
        break;
      case 0:
        /* code */
        deg_to_rad(q_result_aubo,q_right_0);
        break;
      default:
        break;
    }
    
  
    MatrixXd q_left_bTe(4,4);
    aubo_forward(q_left_bTe,q_result_aubo);
    MatrixXd q_left_bTc_left1(4,4);
    q_left_bTc_left1=q_left_bTe*Matrix_eTc;
   for(size_t i=0;i<cloud_filtered_1->points.size();++i)
    {
      // std::cout<< "num:"<<i<<"  "<< cloud_0->points[i].x<<"  "<<cloud_0->points[i].y<<" "<<cloud_0->points[i].z<<"  "<< cloud_0->points[i].rgb<<std::endl;
      
      Matrix_pcl_one_point<<cloud_filtered_1->points[i].x,cloud_filtered_1->points[i].y,cloud_filtered_1->points[i].z,1;
      //for first point
      if(atoi(argv[2])==10)
        Matrix_pcl_new=Matrix_bTc*Matrix_pcl_one_point;
      //for second point
      else
      {
        
      
      Matrix_pcl_new=q_left_bTc_left1*Matrix_pcl_one_point;
      }
      // std::cout<<Matrix_pcl_new<<std::endl;
      pointrgb p;
      p.x=Matrix_pcl_new(0,0);
      p.y=Matrix_pcl_new(1,0);
      p.z=Matrix_pcl_new(2,0);
      p.rgb=cloud_filtered_1->points[i].rgb;
      final->points.push_back(p);
    }
    final->width=1;
    final->height=cloud_filtered_1->points.size();


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::micro> elapsed = end - start;
    std::cout<< "time: "  << elapsed.count()/10000000.0f << "s" << std::endl;
    // std::cout<<"cloud_normals normal size:"<<cloud_normals->size()<<" "<<std::endl;

    pcl::PCDWriter writer;
    std::stringstream sss;
    sss << "cloud_registration_" <<atoi(argv[2])<< ".pcd";
    writer.write<pcl::PointXYZRGB> (sss.str (), *final, false); 
    // writer.write<pcl::PointXYZRGB>("trans_2_base.pcd", *final, false);


    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //创建一个PointIndices结构体指针
    // // 创建分割对象
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // // 可选
    // seg.setOptimizeCoefficients(true); //设置对估计的模型做优化处理
    // // 必选
    // seg.setModelType(pcl::SACMODEL_PLANE);//设置分割模型类别
    // seg.setMethodType(pcl::SAC_RANSAC);//设置使用那个随机参数估计方法
    // seg.setMaxIterations(1000);//迭代次数
    // seg.setDistanceThreshold(0.0018);//设置是否为模型内点的距离阈值


    // std::stringstream ss;
    // ss << "point_to_plane" << ".pcd"; //对每一次的提取都进行了文件保存
    // writer.write<pcl::PointXYZ>(ss.str(), *cloud_cp, false);
    
  return (0);
}