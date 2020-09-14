#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "emPointCloudOperation.h"
#include "emController.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//pcl
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
//yaml
#include <yaml-cpp/yaml.h>
//aubo
#include "../include/aubo_kinematics.h"

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4
typedef pcl::PointXYZ pointxyz;
typedef pcl::PointXYZI pointxyzi;
typedef pcl::PointXYZINormal pointxyzinormal;
typedef pcl::PointXYZRGB pointrgb;
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

using namespace std;

unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
unsigned char ImgBufferGray[IMG_WIDTH*IMG_HEIGHT] = {0};

void *m_Device_1 = NULL;
       
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
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.5, 0.7, 0.9);
    std::cout << "i only run once" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "smart_eye_ros_node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_output", 1);
    ros::Publisher aubo_control_pub = nh.advertise<std_msgs::String> ("/aubo_ros_script/movej", 1);

    ros::Publisher pcl_multipule_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_shortest_path_point_output", 1);
    sensor_msgs::PointCloud2 shortest_path_output;

    sensor_msgs::PointCloud2 output;
    int open_camera_flag=0;
    std::string smarteye_frame_id;
    std::string save_pcd_name;
    int save_to_pcd_flag=0;

    emController *emDemo = new emController();
    
    // pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>),new_pub_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pub_downsample_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),z_filter_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>)
                                        ,cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        final (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_pub (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);

    std::string eye2hand_config_addr;
    MatrixXd TBC_Matrix(4,4);
    int count_pub_joint=0;
    int diffrential_num=0;
    int count_back=0;
    if(ros::param::has("/smarteye_with_viewpoint/diffrential_num"))
    {
        ros::param::get("/smarteye_with_viewpoint/diffrential_num",diffrential_num);
        count_back=diffrential_num;
    }

    auto start = std::chrono::steady_clock::now();
    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		ROS_INFO("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		ros::Rate loop_rate(1);
            pcl::PCDWriter writer;
            
            while (ros::ok())
            {
                if(ros::param::has("/smarteye_with_viewpoint/open_camera_flag"))
                {
                    ros::param::get("/smarteye_with_viewpoint/open_camera_flag",open_camera_flag);
                }else
                {
                    ROS_ERROR("No open_camera_flag parameter,Please check your Launch file\n");
                }
                if(ros::param::has("/smarteye_with_viewpoint/eye2hand_config_addr"))
                {

                    ros::param::get("/smarteye_with_viewpoint/eye2hand_config_addr",eye2hand_config_addr);
                    YAML::Node config = YAML::LoadFile(eye2hand_config_addr);
                    int ServerPort = config["ServerPort"].as<int>();
                    // std::cout<<"ServerPort"<<ServerPort<<std::endl;
                    std::vector<float> tbc_16 = config["TBC"].as<std::vector<float>>();
                    std::vector<float> CalibrationPoint = config["CalibrationPoint"].as<std::vector<float>>();
                    std::vector<float> ViewPointLowestPoint = config["ViewPointLowestPoint"].as<std::vector<float>>();

                    VectorXd q_deg_cali(6);
                    for (size_t i = 0; i < q_deg_cali.size(); i++)
                    {
                        /* code */
                        q_deg_cali(i)=CalibrationPoint[i];

                    }
                    VectorXd q_deg_view_point_lowest(6);
                    for (size_t i = 0; i < q_deg_view_point_lowest.size(); i++)
                    {
                        /* code */
                        q_deg_view_point_lowest(i)=ViewPointLowestPoint[i];

                    }
                    

                    VectorXd q_rad_cali(6);
                    deg_to_rad(q_rad_cali,q_deg_cali);
                    MatrixXd T_q_cali(4,4);
                    aubo_forward(T_q_cali,q_rad_cali);
                    MatrixXd T_view_point_lowest(4,4);

                    VectorXd q_rad_view_point_lowest(6);
                    deg_to_rad(q_rad_view_point_lowest,q_deg_view_point_lowest);                    
                    aubo_forward(T_view_point_lowest,q_rad_view_point_lowest);

                    int count_tbc=0;
                    for (size_t i = 0; i < 4; i++)
                    {
                        /* code */
                        for (size_t j = 0; j < 4; j++)
                        {
                            TBC_Matrix(i,j)=tbc_16[count_tbc];
                            count_tbc+=1;
                        }
                        
                    }
                    //step2 caculate TCE
                    MatrixXd T_C_E(4,4);
                    T_C_E=T_q_cali.inverse()*TBC_Matrix;
                    // std::cout<<"T_C_E"<<T_C_E<<std::endl;
                    // std::cout<<"TBC_Matrix"<<TBC_Matrix<<std::endl;
                    double oblique_take_picture_the_max_height=0.0;

                    if(ros::param::has("/smarteye_with_viewpoint/oblique_take_picture_the_max_height"))
                    {
                        ros::param::get("/smarteye_with_viewpoint/oblique_take_picture_the_max_height",oblique_take_picture_the_max_height);
                        std::cout<<"oblique_take_picture_the_max_height:"<<oblique_take_picture_the_max_height<<std::endl;
                        
                        int open_aubo_flag=0;
                        int go_back_initial_flag=0;
                        float deta_z=0.0;
                        float temp_t23= T_view_point_lowest(2,3);
                        std::cout<<"ros::param::has"<<ros::param::has("/smarteye_with_viewpoint/diffrential_num")<<std::endl;
                        if(ros::param::has("/smarteye_with_viewpoint/diffrential_num"))
                        {
                            ros::param::get("/smarteye_with_viewpoint/diffrential_num",diffrential_num);
                            deta_z=(oblique_take_picture_the_max_height-T_view_point_lowest(2,3))/diffrential_num;
                            MatrixXd q_sol_total(6,diffrential_num+1);
                            for (size_t i = 0; i <= diffrential_num; i++)
                            {
                                VectorXd q_temp_re(6);
                                // std::cout<<T_q_cali(2,3)<<std::endl;
                                T_view_point_lowest(2,3)=temp_t23+deta_z*i;
                                // std::cout<<" T_view_point_lowest(2,3)"<< T_view_point_lowest(2,3)<<std::endl;
                                bool res=GetInverseResult(T_view_point_lowest,q_rad_view_point_lowest,q_temp_re);
                                if (res)
                                {
                                    q_sol_total(0,i) = q_temp_re(0);    q_sol_total(1,i) = q_temp_re(1);
                                    q_sol_total(2,i) = q_temp_re(2);    q_sol_total(3,i) = q_temp_re(3);
                                    q_sol_total(4,i) = q_temp_re(4);    q_sol_total(5,i) = q_temp_re(5);

                                }
                            }
                            std::cout<<"q_sol_total"<<q_sol_total<<"-----"<<q_sol_total.cols()<<std::endl;
                            
                            if(ros::param::has("/smarteye_with_viewpoint/go_back_initial_flag"))
                            {
                                ros::param::get("/smarteye_with_viewpoint/go_back_initial_flag",go_back_initial_flag);
                                if(go_back_initial_flag==1 && count_back>=0){
                                    std::string str1="movej(";
                                    std::string str2="0.0,0.0,0.0,0.0,0.0,0.0";
                                    str1.append(str2);
                                    for (size_t i = 0; i < q_sol_total.rows(); i++)
                                    {
                                        /* code */
                                        std::string str3=std::to_string(q_sol_total.col(count_back)(i));
                                        str1.append(",");
                                        str1.append(str3);

                                    }
                                    str1.append(")");
                                    std_msgs::String msg;
                                    msg.data=str1;
                                    aubo_control_pub.publish(msg);
                                    std::cout<<str1<<std::endl;
                                    std::cout<<"---------------------------------"<<count_back<<std::endl;
                                       
                                    if(count_back==0)
                                    {
                                        ros::param::set("/smarteye_with_viewpoint/go_back_initial_flag",0);
                                        count_back=diffrential_num+1;
                                    }
                                     count_back--;
                                }

                            }else
                            {
                                ROS_INFO("go_back_initial_flag is more than the max point ---\n");
                            }
                            if(ros::param::has("/smarteye_with_viewpoint/open_aubo_flag"))
                            {
                                ros::param::get("/smarteye_with_viewpoint/open_aubo_flag",open_aubo_flag);
                                if(open_aubo_flag==1){
                                    if(q_sol_total.cols()!=0 && count_pub_joint<q_sol_total.cols() && go_back_initial_flag==0){
                                        std::string str1="movej(";
                                        std::string str2="0.0,0.0,0.0,0.0,0.0,0.0";
                                        str1.append(str2);
                                        for (size_t i = 0; i < q_sol_total.rows(); i++)
                                        {
                                            /* code */
                                            std::string str3=std::to_string(q_sol_total.col(count_pub_joint)(i));
                                            str1.append(",");
                                            str1.append(str3);

                                        }
                                        str1.append(")");
                                        std_msgs::String msg;
                                        msg.data=str1;
                                        aubo_control_pub.publish(msg);
                                    }
                                }
                            }else
                            {
                                ROS_INFO("open_aubo_flag is more than the max point ---\n");
                            }
                        

                            //step4 get all cloud from camera
                            if(open_camera_flag==1)
                            {
                                emDemo->emDevStart(0);
                                usleep(1000*1000);
                                
                                cloud->clear();
                                pub_downsample_cloud->clear();
                                cloud_filtered_1->clear();

                                emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
                                convert2PCLPointCloud(emCloud, cloud);

                                std::cerr << "PointCloud after cloud has: "<< cloud->points.size () << " data points." << std::endl;
                                pcl::PassThrough<pcl::PointXYZRGB> pass;
                                pass.setInputCloud (cloud);
                                pass.setFilterFieldName ("z");
                                pass.setFilterLimits (0.20, 1.300); //
                                pass.filter (*z_filter_cloud);

                                pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
                                sor1.setInputCloud (z_filter_cloud);
                                sor1.setLeafSize (0.005f, 0.005f, 0.005f);
                                sor1.filter (*pub_downsample_cloud);
                                std::cerr << "PointCloud after VoxelGrid has: "<< pub_downsample_cloud->points.size () << " data points." << std::endl;
                                //caculate the this point TBC
                                MatrixXd new_T_B_C(4,4);
                                MatrixXd T_B_E_temp(4,4);
                                std::cout<<"-----------------------"<<count_pub_joint<<std::endl;
                                if(count_pub_joint<q_sol_total.cols())
                                {
                                    aubo_forward(T_B_E_temp,q_sol_total.col(count_pub_joint));
                                    new_T_B_C=T_B_E_temp*T_C_E;
                                    // std::cout<<"new_T_B_C---->"<<new_T_B_C<<std::endl;

                                    for (size_t i = 0; i <pub_downsample_cloud->points.size(); i++)
                                    {
                                        MatrixXd cloud_vector_temp_matrix(4,1);
                                        MatrixXd cloud_vector_new_matrix(4,1);
                                        cloud_vector_temp_matrix(0,0)=pub_downsample_cloud->points[i].x;
                                        cloud_vector_temp_matrix(1,0)=pub_downsample_cloud->points[i].y;
                                        cloud_vector_temp_matrix(2,0)=pub_downsample_cloud->points[i].z;
                                        cloud_vector_temp_matrix(3,0)=1;
                                        
                                        cloud_vector_new_matrix=new_T_B_C*cloud_vector_temp_matrix;
                                        // std::cout<<"Point"<<" "<<i<<" "<<cloud_vector_new_matrix<<std::endl;
                                        
                                        pcl::PointXYZRGB p;
                                        p.x=cloud_vector_new_matrix(0,0);
                                        p.y=cloud_vector_new_matrix(1,0);
                                        p.z=cloud_vector_new_matrix(2,0);
                                        p.rgb=pub_downsample_cloud->points[i].rgb;
                                        cloud_filtered_1->points.push_back(p);
                                        pcl::PointXYZRGB p_p;
                                        p_p.x=cloud_vector_new_matrix(0,0);
                                        p_p.y=cloud_vector_new_matrix(1,0);
                                        p_p.z=cloud_vector_new_matrix(2,0);
                                        p_p.rgb=pub_downsample_cloud->points[i].rgb;
                                        cloud_pub->points.push_back(p);
                                        /* code */
                                    }

                                    pcl::toROSMsg(*cloud_pub, output);
                                    usleep(100*1000);
                                    output.header.frame_id = "smarteye_odom";
                                    pcl_pub.publish(output);
                                    

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


                                    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree_extrac (new pcl::search::KdTree<pcl::PointXYZRGB>);
                                    pcl::PointCloud <pcl::Normal>::Ptr normals_region (new pcl::PointCloud <pcl::Normal>);
                                    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
                                    normal_estimator.setSearchMethod (tree_extrac);
                                    normal_estimator.setInputCloud (cloud_filtered_1);
                                    normal_estimator.setKSearch (50);
                                    normal_estimator.compute (*normals_region);

                                    pcl::IndicesPtr indices (new std::vector <int>);


                                    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
                                    reg.setMinClusterSize (10);
                                    reg.setMaxClusterSize (1000000);
                                    reg.setSearchMethod (tree_extrac);
                                    reg.setNumberOfNeighbours (10);
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
                                        shortest_path_output.header.frame_id = "smarteye_odom";
                                        pcl_multipule_pub.publish(shortest_path_output);
                                        
                                        auto end = std::chrono::steady_clock::now();
                                        std::chrono::duration<double, std::micro> elapsed = end - start;
                                        std::cout<< "ALGO --->:time Consuming: "  << elapsed.count()/10000000.0f << " unit(s) " << std::endl;
                                    }
                                }else
                                {
                                    ROS_INFO("count_pub_joint is more than the max point ---\n");
                                    std::cout<<"count_pub_joint"<<" "<<count_pub_joint<<std::endl;
                                }

                                

                                if(ros::param::has("/smarteye_with_viewpoint/smarteye_frame_id"))
                                {
                                    ros::param::get("/smarteye_with_viewpoint/smarteye_frame_id",smarteye_frame_id);
                                    output.header.frame_id = smarteye_frame_id;
                                }else
                                {
                                    output.header.frame_id = "smarteye_odom";
                                }
                                
                                // viewers.showCloud(cloud);
                                // pcl_pub.publish(output);
                                emDemo->emDevStop(0);
                                ros::param::set("/smarteye_with_viewpoint/open_camera_flag",0);

                                count_pub_joint++;
                            }else
                            {
                                ROS_INFO("Please Wait the open parameter!\n");
                            }

                        }
                        else
                        {
                            ROS_ERROR("No diffrential_num parameter,Please check your Launch file\n");
                        }

                    }else
                    {
                        ROS_ERROR("No oblique_take_picture_the_max_height parameter,Please check your Launch file\n");
                    }

                }else
                {
                    ROS_ERROR("No eye2hand_config_addr parameter,Please check your Launch file\n");
                }

                ros::spinOnce();
                loop_rate.sleep();	
			    
			}
        
        }
        else
            ROS_ERROR("Open failed!\n");
    }
    else
        ROS_ERROR("Scan failed!\n");


    return 0;
}
