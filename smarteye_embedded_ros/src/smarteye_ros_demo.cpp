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
    sensor_msgs::PointCloud2 output;
    int open_camera_flag=0;
    std::string smarteye_frame_id;
    std::string save_pcd_name;
    int save_to_pcd_flag=0;
    // test for single capturing
    emController *emDemo = new emController();
    
    // pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pub_downsample_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),z_filter_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);
    
    
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
                    if(save_to_pcd_flag)
                    {
                        if(count>=2){
                            ROS_INFO("Start Save\n");
                            std::cout<<"wocao--->:"<<count<<std::endl;
                            std::stringstream sss;
                            sss << "/data/cloud_save_" <<count-1<< ".pcd";
                            writer.write<pcl::PointXYZRGB> (sss.str (), *pub_downsample_cloud, false); //*

                            
                            // pcl::io::savePCDFileASCII(save_pcd_name,*cloud);
                            // sleep(30);
                            // ros::param::set("/smarteye_ros_demo/save_to_pcd_flag",0);
                            ROS_INFO("End Save\n");
                        }
                    }
                    
                    if(ros::param::has("/smarteye_ros_demo/smarteye_frame_id"))
                    {
                        ros::param::get("/smarteye_ros_demo/smarteye_frame_id",smarteye_frame_id);
                        output.header.frame_id = smarteye_frame_id;
                    }else
                    {
                        output.header.frame_id = "smarteye_odom";
                    }
                    
                    // viewers.showCloud(cloud);
                    // pcl_pub.publish(output);
                    emDemo->emDevStop(0);
                    ros::param::set("/smarteye_ros_demo/open_camera_flag",0);
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

    getchar();
    return 0;
}
