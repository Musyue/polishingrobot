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
#include <pcl/io/pcd_io.h>

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4


using namespace std;

unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
unsigned char ImgBufferGray[IMG_WIDTH*IMG_HEIGHT] = {0};

void *m_Device_1 = NULL;
static bool bflag = false;

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
        printf("recv ok----. %d, %ld\n", recvBufID, recvBufID_Old);
        memcpy(ImgBuffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
        bflag = true;
    }
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.5, 0.7, 0.9);
    std::cout << "i only run once" << std::endl;
}

int main(int argc,char*argv[])
{
    // test for single capturing
    emController *emDemo = new emController();
    
    pcl::visualization::CloudViewer viewers("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud_EM::Ptr emCloud(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);
    int flag=1;
    
    uint32_t extime=0;
    // uint32_t *p=NULL;
    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		printf("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		
            emDemo->emGetExposureTime(0,extime);
            emDemo->emSetExposureTime(0,12000,2);//12000us 1.1 meter
            emDemo->emGetExposureTime(0,extime);
            emDemo->emDevStart(0);
            std::cout<<"exspore time------>"<<extime<<std::endl;
            while(1)
            {
                // std::cout<<"exspore time------>"<<extime<<std::endl;
                usleep(500*1000);
                if(bflag)
                {
                    bflag = false;
                    cloud->clear();
                    emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
                    convert2PCLPointCloud(emCloud, cloud);
                    
                    // if(flag>=2)
                    // {
                    //     printf("start save\n");
                    //     std::stringstream sss;
                    //     sss << "/data/ros/pcldata2/pcd_" <<argv[1]<< ".pcd";
                    //     pcl::io::savePCDFileASCII(sss.str (),*cloud);
                    //     printf("end save\n");
                    // }
                    viewers.showCloud(cloud);	
                    emDemo->emDevStart(0);
                    //emDemo->emDevStop(0);
                    
                }
		        usleep(10*1000);
                //flag+=1;
			}
            // while (1)
            // {
            //     /* code */
            // }
            
            while (!viewers.wasStopped()) {}	
        }
        else
            printf("Open failed!\n");
    }
    else
        printf("Scan failed!\n");

    getchar();
    return 0;
}
