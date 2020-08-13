#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>

#include "emPointCloudOperation.h"
#include "emController.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

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
        printf("recv ok. %ld\n", recvBufID_Old);
        ///////////////////////////////////////////
        //ADD YOUR TODO CODE
      memcpy(ImgBuffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
    }
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.5, 0.7, 0.9);
    std::cout << "i only run once" << std::endl;
}

int main()
{
    // test for single capturing
    emController *emDemo = new emController();
    
    pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);
    
    
    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		printf("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		
            while(1)
            {
                emDemo->emDevStart(0);
                usleep(1000*1000);
		        cloud->clear();
		        emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
		        convert2PCLPointCloud(emCloud, cloud);
			    viewers.showCloud(cloud);	
			    emDemo->emDevStop(0);
			}
            //while (!viewers.wasStopped()) {}	
        }
        else
            printf("Open failed!\n");
    }
    else
        printf("Scan failed!\n");

    getchar();
    return 0;
}
