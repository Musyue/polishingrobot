#ifndef SMARTEYE_WITH_VIEWPOINT_AND_ENV_EXPLORITION_H
#define SMARTEYE_WITH_VIEWPOINT_AND_ENV_EXPLORITION_H
//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
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
#include "smarteye_embedded_ros/aubo_kinematics.h"
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
unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
unsigned char ImgBufferGray[IMG_WIDTH*IMG_HEIGHT] = {0};
void *m_Device_1 = NULL;
namespace smarteye_viewpoint_and_env_explorition {
    class SmarteyeViewpointEnvExp {
        public:
            SmarteyeViewpointEnvExp();
            ~SmarteyeViewpointEnvExp() {};
            void OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM* pFrame);
            int cmp(const void *pleft, const void *pright);
            void registerNodeHandle(ros::NodeHandle& _nh);
            void registerPubSub();

        private:
            ros::Publisher downsample_pub;
            ros::Publisher aubo_movej_pub;
            ros::Publisher aubo_movet_pub;
            ros::Publisher aubo_movel_pub;
            ros::NodeHandle nh;
    }
}

#endif