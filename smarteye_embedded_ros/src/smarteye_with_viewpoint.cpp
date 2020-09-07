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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
//yaml
#include <yaml-cpp/yaml.h>
//aubo
#include "../include/aubo_kinematics.h"
#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4


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
    sensor_msgs::PointCloud2 output;
    int open_camera_flag=0;
    std::string smarteye_frame_id;
    std::string save_pcd_name;
    int save_to_pcd_flag=0;

    emController *emDemo = new emController();
    
    // pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pub_downsample_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),z_filter_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);

    std::string eye2hand_config_addr;
    MatrixXd TBC_Matrix(4,4);


    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		ROS_INFO("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		ros::Rate loop_rate(1);
            // while(1)
            int count=0;
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
                    std::cout<<"ServerPort"<<ServerPort<<std::endl;
                    std::vector<float> tbc_16 = config["TBC"].as<std::vector<float>>();
                    std::vector<float> view_point_joint_deg = config["ViewPointStart"].as<std::vector<float>>();
                    std::vector<float> ViewPointStart = config["ViewPointStart"].as<std::vector<float>>();

                    VectorXd q_deg_start(6);
                    for (size_t i = 0; i < q_deg_start.size(); i++)
                    {
                        /* code */
                        q_deg_start(i)=view_point_joint_deg[i];

                    }
                    

                    VectorXd q_rad_start(6);
                    deg_to_rad(q_rad_start,q_deg_start);
                    MatrixXd T_q_start(4,4);
                    aubo_forward(T_q_start,q_rad_start);
                    std::cout<<"wocao--"<<q_rad_start<<std::endl;
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
                    double oblique_take_picture_the_max_height=0.0;

                    if(ros::param::has("/smarteye_with_viewpoint/oblique_take_picture_the_max_height"))
                    {
                        ros::param::get("/smarteye_with_viewpoint/oblique_take_picture_the_max_height",oblique_take_picture_the_max_height);
                        std::cout<<"oblique_take_picture_the_max_height:"<<oblique_take_picture_the_max_height<<std::endl;
                        int diffrential_num=0.0;
                        float deta_z=0.0;
                        float temp_t23= T_q_start(2,3);
                        if(ros::param::has("/smarteye_with_viewpoint/diffrential_num"))
                        {
                            ros::param::get("/smarteye_with_viewpoint/diffrential_num",diffrential_num);
                            deta_z=(oblique_take_picture_the_max_height-T_q_start(2,3))/diffrential_num;
                            for (size_t i = 0; i < diffrential_num; i++)
                            {
                                VectorXd q_temp_re(6);
                                // std::cout<<T_q_start(2,3)<<std::endl;
                                T_q_start(2,3)=temp_t23+deta_z*i;
                                // std::cout<<T_q_start(2,3)<<std::endl;
                                bool res=GetInverseResult(T_q_start,q_rad_start,q_temp_re);
                                if (res)
                                {
                                    std::string str1="movej(";
                                    std::string str2="0.0,0.0,0.0,0.0,0.0,0.0";
                                    str1.append(str2);
                                    for (size_t i = 0; i < q_temp_re.size(); i++)
                                    {
                                        /* code */
                                        std::string str3=std::to_string(q_temp_re(i));
                                        str1.append(",");
                                        str1.append(str3);

                                    }
                                    str1.append(")");
                                    // aubo_control_pub.publish(str1);  
                                    std::cout<<"q_temp_re:"<<str1<<std::endl;
                                    std::stringstream write_str;
                                    std_msgs::String msg;
                                    

                                    write_str << "movej(" <<"0.0,0.0,0.0,0.0,0.0,0.0"<<q_temp_re<<")";//sss.str ()
                                    // std::cout<<"q_temp_re:"<<write_str.str()<<std::endl;
                                    msg.data=str1;//write_str.str();
                                    aubo_control_pub.publish(msg);
                                    //std::cout<<"q_temp_re:"<<q_temp_re<<std::endl;
                                }
                            }
                        }else
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
                if(open_camera_flag==1)
                {
                    emDemo->emDevStart(0);
                    usleep(1000*1000);
                    
                    cloud->clear();
                    emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
                    convert2PCLPointCloud(emCloud, cloud);
                    // pcl::io::savePCDFileASCII("/data/yue_test.pcd",*cloud);
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
                    
                    pcl::toROSMsg(*pub_downsample_cloud, output);
                    output.header.frame_id = "smarteye_odom";
                    pcl_pub.publish(output);

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
                    count++;
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
