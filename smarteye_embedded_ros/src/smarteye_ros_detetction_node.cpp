#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include "emPointCloudOperation.h"
#include "emController.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4



typedef pcl::PointXYZ pointxyz;
typedef pcl::PointXYZI pointxyzi;
typedef pcl::PointXYZINormal pointxyzinormal;
typedef pcl::PointXYZRGB pointrgb;


unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
unsigned char ImgBufferGray[IMG_WIDTH*IMG_HEIGHT] = {0};

void *m_Device_1 = NULL;
typedef struct mypointxyzinormal
{
	float x;
	float y;
	float z;
    float normal_x;
    float normal_y;
    float normal_z;
    float curvature;
    float distance;
}mypointxyzinormal_t;
int cmp(const void *pleft, const void *pright){
	mypointxyzinormal_t *p1=(mypointxyzinormal_t *)pleft;
	mypointxyzinormal_t *p2=(mypointxyzinormal_t *)pright;
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
void OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM* pFrame)
{
    static uint64_t recvBufID_Old = 0;
    uint64_t recvBufID = pFrame->nFrameID;

    if(recvBufID_Old != recvBufID)
    {
        recvBufID_Old = pFrame->nFrameID;
        // printf("recv ok. %ld\n", recvBufID_Old);
        ///////////////////////////////////////////
        //ADD YOUR TODO CODE
        printf("recv ok----. %ld\n", recvBufID_Old);
      memcpy(ImgBuffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
    }
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.5, 0.7, 0.9);
    std::cout << "i only run once" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "smart_eye_ros_node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_downsample_output", 1);
    sensor_msgs::PointCloud2 downsample_output;
    ros::Publisher pcl_multipule_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_shortest_path_point_output", 1);
    sensor_msgs::PointCloud2 shortest_path_output;
    int open_camera_flag=0;
    std::string smarteye_frame_id;
    std::string save_pcd_name;
    int save_to_pcd_flag=0;
    // test for single capturing
    emController *emDemo = new emController();
    
    // pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>)
                                        ,cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        final (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);
    auto start = std::chrono::steady_clock::now();
    
    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		ROS_INFO("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		ros::Rate loop_rate(1);

            // while(1)
            while (ros::ok())
            {
                if(ros::param::has("/smarteye_ros_demo/open_camera_flag"))
                {

                    ros::param::get("/smarteye_ros_demo/open_camera_flag",open_camera_flag);
                }else
                {
                    ROS_ERROR("No open_camera_flag parameter,Please check your Launch file\n");
                }
                if(ros::param::has("/smarteye_ros_demo/save_to_pcd_flag"))
                {

                    ros::param::get("/smarteye_ros_demo/save_to_pcd_flag",save_to_pcd_flag);
                }else
                {
                    ROS_ERROR("No save_to_pcd_flag parameter,Please check your Launch file\n");
                }
                if(ros::param::has("/smarteye_ros_demo/save_pcd_name"))
                {

                    ros::param::get("/smarteye_ros_demo/save_pcd_name",save_pcd_name);
                }else
                {
                    ROS_ERROR("No save_pcd_name parameter,Please check your Launch file\n");
                }
                if(open_camera_flag==1)
                {
                    emDemo->emDevStart(0);
                    usleep(1000*1000);
                    cloud->clear();
                    emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
                    convert2PCLPointCloud(emCloud, cloud);

                    // pcl::toROSMsg(*cloud, output);
                    if(save_to_pcd_flag)
                    {
                        ROS_INFO("Start Save\n");
                        pcl::io::savePCDFileASCII(save_pcd_name,*cloud);
                        // sleep(30);
                        // ros::param::set("/smarteye_ros_demo/save_to_pcd_flag",0);
                        ROS_INFO("End Save\n");
                    }
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

                    pcl::toROSMsg(*cloud_filtered_1, downsample_output);
                    pcl_pub.publish(downsample_output);


                    pcl::PCDWriter writer;

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

                    //You need to change to server parameter
                    seg.setDistanceThreshold(0.0018);//设置是否为模型内点的距离阈值

   
                    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                    int i = 0, nr_points = (int)cloud_filtered_1->points.size();

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

                        cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << endl;
                        extract.setNegative(true);
                        extract.filter(*cloud_f);
                        std::cerr << "PointCloud Extract Points: "
                            << cloud_f->points.size () << " data points." << std::endl;

                        cloud_filtered_1.swap(cloud_f);//经cloud_filtered与cloud_f交换

                        i++;
                    }

                    cout << "cloud_filtered: " << cloud_filtered_1->size() << endl;


                    // pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
                    // ne.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
                    // ne.setInputCloud (cloud_filtered_1);

                    // // Create an empty kdtree representation, and pass it to the normal estimation object.
                    // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
                    // ne.setSearchMethod (tree);

                    // // Output datasets
                    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

                    // // Use all neighbors in a sphere of radius 5cm
                    // ne.setRadiusSearch (0.1);
                    // // ne.setIndices();
                    // // Compute the features
                    // ne.compute (*cloud_normals);




                    // std::stringstream sss;
                    // sss << "cloud_filtered_1" << ".pcd";
                    // writer.write<pcl::PointXYZRGB> (sss.str (), *cloud_filtered_1, false); //*

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
                        // //IO
                        // std::stringstream ss;
                        // ss << "cloud_cluster_" << i << ".pcd";
                        // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_pp, false); //*


                        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> nee;
                        nee.setNumberOfThreads(12);  // 
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
                        // //IO
                        // std::stringstream sss;
                        // sss << "cloud_cp_" << i << ".pcd";
                        // writer.write<pcl::PointXYZINormal> (sss.str (), *cloud_cp, false); //*

                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr sort_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
                        mypointxyzinormal_t parray[cloud_cp->points.size()];

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
                        
                        qsort(parray, cloud_cp->points.size(), sizeof(mypointxyzinormal), cmp);

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
                        // std::stringstream ssss;
                        // ssss << "sort_cloud_" << i << ".pcd";
                        // writer.write<pcl::PointXYZINormal> (ssss.str (), *sort_cloud, false); //*

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

                        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
                        ROS_INFO("Here Will us greedy alogrithm to search the shortest path!");
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

                        pcl::toROSMsg(*pub_cloud, shortest_path_output);
                        pcl_pub.publish(shortest_path_output);
                        shortest_path_output.header.frame_id = "smart_shortest_odom";
                        auto end = std::chrono::steady_clock::now();
                        std::chrono::duration<double, std::micro> elapsed = end - start;
                        std::cout<< "ALGO --->:time Consuming: "  << elapsed.count()/10000000.0f << " unit(s) " << std::endl;

                    }
                    if(ros::param::has("/smarteye_ros_demo/smarteye_frame_id"))
                    {
                        ros::param::get("/smarteye_ros_demo/smarteye_frame_id",smarteye_frame_id);
                        downsample_output.header.frame_id = smarteye_frame_id;
                    }else
                    {
                        downsample_output.header.frame_id = "smarteye_odom";
                    }
                    
                    // viewers.showCloud(cloud);
                    // pcl_pub.publish(output);

                    emDemo->emDevStop(0);
                    ros::param::set("/smarteye_ros_demo/open_camera_flag",0);
                    
                }else
                {
                    ROS_INFO("Please Wait the open parameter!\n");
                }
                


                

                ros::spinOnce();
                loop_rate.sleep();	
			    
			}
            //while (!viewers.wasStopped()) {}	
        }
        else
            ROS_ERROR("Open failed!\n");
    }
    else
        ROS_ERROR("Scan failed!\n");

    return 0;
}
