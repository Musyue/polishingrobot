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
#include <Eigen/Core>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include<iostream>
typedef pcl::PointXYZ pointxyz;

typedef pcl::PointXYZRGB pointrgb;
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2,pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3_norm)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255,240,245);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 255, 0, 0);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(cloud2, 	176,196,222);

//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(cloud3, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "after rasanc");

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2,"initial cloud");
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud3, single_color2, "normal cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "after rasanc");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "normal cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "initial cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud3_norm, cloud_normals,20,0.03,"cloud_normals");
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  return (viewer);
}
struct mypoint
{
	float x;
	float y;
	float z;
}parray[10000];
int cmp(const void *arg1, const void *arg2){
	mypoint *starg1 = (mypoint*)arg1;
	mypoint *starg2 = (mypoint*)arg2;

	return ( starg1->z - starg2->z );
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
                                        final (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PCDReader reader;
    std::vector<int> inliers;//用于存放合群点的vector
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr aftercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud_0)==-1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    // printf("woqu\n");
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
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (.02);
    ransac.computeModel();
    ransac.getInliers(inliers);
    std::vector<int> tmp;
    Eigen::VectorXf coeff;
    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *final);
    ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
    std::cout<<"coeff "<<coeff[0]<<" "<<coeff[1]<<" "<<coeff[2]<<" "<<coeff[3]<<std::endl;
    ransac.getModel(tmp);

    //remove some noise point use this class
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (final);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1.0);
    sor.setNegative (true);
    sor.filter (*cloud_filtered_1);

    //use Voxelgrid downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
    sor1.setInputCloud (cloud_filtered_1);
    sor1.setLeafSize (0.02f, 0.02f, 0.02f);
    sor1.filter (*cloud_filtered_1);
    std::cerr << "PointCloud after VoxelGrid has: "
            << cloud_filtered_1->points.size () << " data points." << std::endl;

  // for (size_t i = 0; i < cloud_filtered_1->points.size (); i++)
  // {
  //   /* code */
  //   std::cout<< "After VoxelGrid:"<<i<<"  "<< cloud_filtered_1->points[i].x<<"  "<<cloud_filtered_1->points[i].y<<" "<<cloud_filtered_1->points[i].z<<"  "<< cloud_filtered_1->points[i].rgb<<std::endl;
  // }

  //sort to find the largest point
  pcl::PointCloud<pcl::PointXYZ>::Ptr sort_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if(cloud_filtered_1->points.size ()<15000)
  {
    for (int i = 0; i < cloud_filtered_1->points.size(); i++)
    {
      parray[i].x = cloud_filtered_1->points[i].x;
      parray[i].y = cloud_filtered_1->points[i].y;
      parray[i].z = cloud_filtered_1->points[i].z;
    }
  }
  qsort(parray, cloud_filtered_1->points.size(), sizeof(mypoint), cmp);
  // for (size_t i = cloud_filtered_1->points.size(); i >cloud_filtered_1->points.size(); i--)
  for (size_t i = 0; i <cloud_filtered_1->points.size(); i++)
  {
    pointxyz p;
    p.x=parray[i].x;
    p.y=parray[i].y;
    p.z=parray[i].z;
    sort_cloud->points.push_back(p);
    /* code */
  }
    sort_cloud->width=1;
    sort_cloud->height=cloud_filtered_1->points.size();
    //normal estimation
    
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
    ne.setInputCloud (sort_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 5cm
    ne.setRadiusSearch (0.05);

    // Compute the features
    ne.compute (*cloud_normals);
    // for (size_t i = 0; i < cloud_normals->size(); i++)
    // {
       std::cout<<"cloud_normals normal size:"<<cloud_normals->size()<<" "<<std::endl;
        /* code */
        // std::cout<<cloud_normals->points[i].normal<<" "
        // <<cloud_normals->points[i].normal_x<<" "
        // <<cloud_normals->points[i].normal_y<<" "
        // <<cloud_normals->points[i].normal_z<<" "
        // <<std::endl;
    // }
    
    pcl::PCDWriter writer;

    viewer = customColourVis(cloud_filtered_1,cloud_filtered,cloud_normals,sort_cloud);
    writer.write ("after_rasanc.pcd", *cloud_filtered_1, false);
    writer.write ("cloud_rasanc.pcd", *sort_cloud, false);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  return (0);
}