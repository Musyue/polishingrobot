#ifndef _EM_POINTCLOUDOPT_H_
#define _EM_POINTCLOUDOPT_H_

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/crop_box.h>

#include "emCommon_type.h"

void convert2PCLPointCloud(PointCloud_EM_Ptr pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud);

void convert2PCLPointCloud(PointCloud_EM_Ptr pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud)
{
    if (NULL == pointCloud || NULL == pcl_cloud) {
        return;
    }

    int width = pointCloud->width;
    int height = pointCloud->height;
    pcl_cloud->width = width;
    pcl_cloud->height = height;
    pcl_cloud->resize(width*height);

//#pragma omp_set_num_threads(3);
//#pragma omp parallel for
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            (*pcl_cloud)(w, h).r = (*pointCloud)(w, h).r;
            (*pcl_cloud)(w, h).g = (*pointCloud)(w, h).g;
            (*pcl_cloud)(w, h).b = (*pointCloud)(w, h).b;
            (*pcl_cloud)(w, h).x = (*pointCloud)(w, h).x/1000.0f;
            (*pcl_cloud)(w, h).y = (*pointCloud)(w, h).y/1000.0f;
            (*pcl_cloud)(w, h).z = (*pointCloud)(w, h).z/1000.0f;
        }
    }
}

#endif
