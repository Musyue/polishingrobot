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
#include <pcl/common/common.h>

//for ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//
typedef pcl::PointXYZ pointxyz;
typedef pcl::PointXYZI pointxyzi;
typedef pcl::PointXYZINormal pointxyzinormal;
using namespace std;
typedef pcl::PointXYZRGB pointrgb;
typedef struct mypoint
{
	float x;
	float y;
	float z;
  float normal_x;
  float normal_y;
  float normal_z;
  float curvature;
  float distance;
}mypoint_t;
int cmp(const void *pleft, const void *pright){
	mypoint_t *p1=(mypoint_t *)pleft;
	mypoint_t *p2=(mypoint_t *)pright;
	if(p1->distance>p2->distance)
	{
		return 1;
	}else if(p1->distance<p2->distance)
	{
		return -1;
	}else{
		return 0;
	}
}
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

    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud_filtered_1)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    // printf("woqu\n");
    ros::init (argc, argv, "smart_eye_ros_node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_output", 1);
    sensor_msgs::PointCloud2 output;
    ros::Rate loop_rate(1);
    auto start = std::chrono::steady_clock::now();

    // for(size_t i=0;i<cloud_0->points.size();++i)
    // {
    //   // std::cout<< "num:"<<i<<"  "<< cloud_0->points[i].x<<"  "<<cloud_0->points[i].y<<" "<<cloud_0->points[i].z<<"  "<< cloud_0->points[i].rgb<<std::endl;
    //   pointrgb p;
    //   p.x=(cloud_0->points[i].x);
    //   p.y=(cloud_0->points[i].y);
    //   p.z=(cloud_0->points[i].z);
    //   p.rgb=cloud_0->points[i].rgb;
    //   cloud->points.push_back(p);
    // }
    // cloud->width=1;
    // cloud->height=cloud_0->points.size();
    
    // pause();
    //set filter
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (0.50, 1.100);
    // pass.filter (*cloud_filtered_2);

    // pcl::PassThrough<pcl::PointXYZRGB> passx;
    // passx.setInputCloud (cloud_filtered_2);
    // passx.setFilterFieldName ("x");
    // // passx.setFilterLimits (-0.43, 0.43);
    // passx.setFilterLimits (-0.40, 0.40);
    // passx.filter (*cloud_filtered_3);

    // pcl::PassThrough<pcl::PointXYZRGB> passy;
    // passy.setInputCloud (cloud_filtered_3);
    // passy.setFilterFieldName ("y");
    // passy.setFilterLimits (-0.20, 0.20);
    // // passy.setFilterLimits (-0.22, 0.22);
    // passy.filter (*cloud_filtered);

    // std::cerr << "PointCloud after filtering has: "
    //         << cloud_filtered->points.size () << " data points." << std::endl;
    // pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
    // sor1.setInputCloud (cloud_filtered);
    // sor1.setLeafSize (0.003f, 0.003f, 0.003f);
    // sor1.filter (*cloud_filtered_1);
    // std::cerr << "PointCloud after VoxelGrid has: "
    //         << cloud_filtered_1->points.size () << " data points." << std::endl;


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
    seg.setDistanceThreshold(0.0017);//设置是否为模型内点的距离阈值

    // 创建滤波器对象
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int i = 0, nr_points = (int)cloud_filtered_1->points.size();
    // 当还多于30%原始点云数据时
    float na,nb,nc,nd;
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
        na=coefficients->values[0];
        nb=coefficients->values[1];
        nc=coefficients->values[2];
        nd=coefficients->values[3];
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

        cloud_filtered_1.swap(cloud_f);//经cloud_filtered与cloud_f交换

        i++;
    }

    cout << "cloud_filtered: " << cloud_filtered_1->size() << endl;


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
    // ne.setIndices();
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

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree_extrac (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals_region (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree_extrac);
    normal_estimator.setInputCloud (cloud_filtered_1);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals_region);

    pcl::IndicesPtr indices (new std::vector <int>);


    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree_extrac);
    reg.setNumberOfNeighbours (50);
    reg.setInputCloud (cloud_filtered_1);
    //reg.setIndices (indices);
    reg.setInputNormals (normals_region);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr last_out_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    if(clusters.size ()>0)
    {
    for (size_t i = 0; i < clusters.size (); i++)
    {
      /* code */
      std::cout <<i<<" cluster has " << clusters[i].indices.size () << " points." << std::endl;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pp(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*cloud_filtered_1, clusters[i].indices, *cloud_pp);
      //IO
      std::stringstream ss;
      ss << "cloud_cluster_" << i << ".pcd";
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_pp, false); //*


      pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> nee;
      nee.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
      nee.setInputCloud (cloud_pp);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      nee.setSearchMethod (tree_normal);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1 (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 5cm
      nee.setRadiusSearch (0.05);
      // ne.setIndices();
      // Compute the features
      nee.compute (*cloud_normals_1);

      pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cp(new pcl::PointCloud<pcl::PointXYZINormal>);
      for(size_t i=0;i<cloud_pp->points.size();++i)
      {
        // std::cout<< "num:"<<i<<"  "<< cloud_0->points[i].x<<"  "<<cloud_0->points[i].y<<" "<<cloud_0->points[i].z<<"  "<< cloud_0->points[i].rgb<<std::endl;
        pointxyzinormal p;
        p.x=(cloud_pp->points[i].x);
        p.y=(cloud_pp->points[i].y);
      //   p.z=(-nd-na*(cloud_filtered->points[i].x)-nb*(cloud_filtered->points[i].y))/nc-(cloud_filtered->points[i].z);
        p.z=(cloud_pp->points[i].z);
        p.intensity=(fabs(na*(cloud_pp->points[i].x)+nb*(cloud_pp->points[i].y)+nc*(cloud_pp->points[i].z)+nd)/sqrt(na*na+nb*nb+nc*nc));
        p.normal_x=cloud_normals_1->points[i].normal_x;
        p.normal_y=cloud_normals_1->points[i].normal_y;
        p.normal_z=cloud_normals_1->points[i].normal_z;
        p.curvature=cloud_normals_1->points[i].curvature;
        cloud_cp->points.push_back(p);
      }
      cloud_cp->width=1;
      cloud_cp->height=cloud_pp->points.size();
      //IO
      std::stringstream sss;
      sss << "cloud_cp_" << i << ".pcd";
      writer.write<pcl::PointXYZINormal> (sss.str (), *cloud_cp, false); //*

      pcl::PointCloud<pcl::PointXYZINormal>::Ptr sort_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
      mypoint_t parray[cloud_cp->points.size()];

      for (int i = 0; i < cloud_cp->points.size(); i++)
      {
        parray[i].x = cloud_cp->points[i].x;
        parray[i].y = cloud_cp->points[i].y;
        parray[i].z = cloud_cp->points[i].z;
        parray[i].distance=cloud_cp->points[i].intensity;
        parray[i].normal_x = cloud_cp->points[i].normal_x;
        parray[i].normal_y = cloud_cp->points[i].normal_y;
        parray[i].normal_z = cloud_cp->points[i].normal_z;
        parray[i].curvature=cloud_cp->points[i].curvature;

      }
      
      qsort(parray, cloud_cp->points.size(), sizeof(mypoint), cmp);

      // for (size_t i = cloud_filtered_1->points.size(); i >cloud_filtered_1->points.size(); i--)
      for (size_t i = 0; i <cloud_cp->points.size(); i++)
      {
        pointxyzinormal p;
        p.x=parray[i].x;
        p.y=parray[i].y;
        p.z=parray[i].z;
        p.intensity=parray[i].distance;
        p.normal_x=parray[i].normal_x;
        p.normal_y=parray[i].normal_y;
        p.normal_z=parray[i].normal_z;
        p.curvature=parray[i].curvature;
        sort_cloud->points.push_back(p);
        /* code */
      }
      sort_cloud->width=1;
      sort_cloud->height=cloud_cp->points.size();
      //IO 
      std::stringstream ssss;
      ssss << "sort_cloud_" << i << ".pcd";
      writer.write<pcl::PointXYZINormal> (ssss.str (), *sort_cloud, false); //*

      //
      std::cout<<" This cluster The Highest Point->"<<sort_cloud->points[cloud_cp->points.size()-1].x<< " "
      <<sort_cloud->points[cloud_cp->points.size()-1].y<< " "
      <<sort_cloud->points[cloud_cp->points.size()-1].z<< " Z distance >>"
      <<sort_cloud->points[cloud_cp->points.size()-1].intensity<< " "
      <<"Normal: "
      <<sort_cloud->points[cloud_cp->points.size()-1].normal_x<< " "
      <<sort_cloud->points[cloud_cp->points.size()-1].normal_y<< " "
      <<sort_cloud->points[cloud_cp->points.size()-1].normal_z<< " "
      <<sort_cloud->points[cloud_cp->points.size()-1].curvature<< " "
      <<std::endl;
      pointxyzinormal p_last;
      p_last.x=sort_cloud->points[cloud_cp->points.size()-1].x;
      p_last.y=sort_cloud->points[cloud_cp->points.size()-1].y;
      p_last.z=sort_cloud->points[cloud_cp->points.size()-1].z;
      p_last.intensity=sort_cloud->points[cloud_cp->points.size()-1].intensity;
      p_last.normal_x=sort_cloud->points[cloud_cp->points.size()-1].normal_x;
      p_last.normal_y=sort_cloud->points[cloud_cp->points.size()-1].normal_y;
      p_last.normal_z=sort_cloud->points[cloud_cp->points.size()-1].normal_z;
      p_last.curvature=sort_cloud->points[cloud_cp->points.size()-1].curvature;
      last_out_cloud->points.push_back(p_last);
    }
    // last_out_cloud->width=1;
    // last_out_cloud->height=cloud_cp->points.size();
    int n=last_out_cloud->size();
    int ii,jj,kk,ll;
    int S[n];
    float D[n][n];//Store the distance between the two node
    float sum=0;//Store the receive cities the shortest path
    float Dtemp;//
    int flag;//visitor flag,if visiting 1,others 0
    for (size_t t = 0; t < n; t++)
    {
      /* code */
      for (size_t iii = 0; iii < n; iii++)
      {
        /* code */
        if(t!=iii)
        {
          D[t][iii]=sqrt(pow(last_out_cloud->points[t].x-last_out_cloud->points[iii].x,2)+pow(last_out_cloud->points[t].y-last_out_cloud->points[iii].y,2)+pow(last_out_cloud->points[t].z-last_out_cloud->points[iii].z,2));
          // std::cout<<"Points"<<" "<<t<<"-"<<iii<<" "<<D[t][iii]<<" "<<std::endl;
        }
      }
      
    }
    ii=1;
    S[0]=0;
    do
    {
        /* code */
        kk=1;Dtemp=1000000.0f;
        do
        {
            /* code */
            ll=0;flag=0;
            do
            {
                /* code */
                if(S[ll]==kk)
                {
                    flag=1;
                    break;
                }else
                {
                    ll++;
                }
                
            } while (ll<ii);
            if (flag==0 && D[kk][S[ii-1]]<Dtemp)
            {
                /* code */
                jj=kk;
                Dtemp=D[kk][S[ii-1]];
            }
            kk++;
            
        } while (kk<n);
        S[ii]=jj;
        ii++;
        sum+=Dtemp;
    } while (ii<n);
    sum+=D[0][jj];

    for (int j = 0; j < n; j++)
    {
        /* code */
        std::cout<<S[j]<<std::endl;
    }
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pub_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    std::cout<<"TSP Solution with the minimal path Sum:"<<sum<<std::endl;
    for (size_t i = 0; i < last_out_cloud->size(); i++)
    {
      /* code */
         std::cout<<" This cluster The Highest Point->"<<last_out_cloud->points[S[i]].x<< " "
      <<last_out_cloud->points[S[i]].y<< " "
      <<last_out_cloud->points[S[i]].z<< " Z distance >>"
      <<last_out_cloud->points[S[i]].intensity<< " "
      <<"Normal: "
      <<last_out_cloud->points[S[i]].normal_x<< " "
      <<last_out_cloud->points[S[i]].normal_y<< " "
      <<last_out_cloud->points[S[i]].normal_z<< " "
      <<last_out_cloud->points[S[i]].curvature<< " "
      <<std::endl;
      pointxyzinormal p_pub;
      p_pub.x=last_out_cloud->points[S[i]].x;
      p_pub.y=last_out_cloud->points[S[i]].y;
      p_pub.z=last_out_cloud->points[S[i]].z;
      p_pub.intensity=last_out_cloud->points[S[i]].intensity;
      p_pub.normal_x=last_out_cloud->points[S[i]].normal_x;
      p_pub.normal_y=last_out_cloud->points[S[i]].normal_y;
      p_pub.normal_z=last_out_cloud->points[S[i]].normal_z;
      p_pub.curvature=last_out_cloud->points[S[i]].curvature;
      pub_cloud->points.push_back(p_pub);
    }

    pub_cloud->width=1;
    pub_cloud->height=last_out_cloud->points.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pub_cloud, output);
    
    // std::cout << "First cluster has " << clusters[0]. indices.size () << " points." << std::endl;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pp(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::copyPointCloud(*cloud_filtered_1, clusters[0].indices, *cloud_pp);


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->initCameraParameters();
    // viewer->addCoordinateSystem (0.5);
    int v1(0);
    viewer->createViewPort(0, 0, 0.25, 1, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color1(colored_cloud, 255, 0, 0);
    viewer->addPointCloud(colored_cloud, "colored_cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "colored_cloud");
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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> color5(last_out_cloud, 255, 0, 0);
    viewer->addPointCloud(last_out_cloud, color5,"cloud_statical_1", v4);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_statical_1");

    viewer->addPointCloud(cloud_filtered, "cloud_statical", v4);
    // viewer->addCoordinateSystem (0.5);
   // viewer->addCoordinateSystem();//添加坐标系

    viewer->spin();
        		
            // while(1)
    int flag_view=0;
    while (ros::ok()){
      pcl_pub.publish(output);
      ros::spinOnce();
      loop_rate.sleep();	
      if(flag_view==0)
        {
          viewer->spin();
          flag_view=1;
        }
      }
    }else
    {
      std::cout<<"There is no cluster find,that's mean this wall is so smooth"<<std::endl;
    }
    
  return (0);
}