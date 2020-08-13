#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>

#include "emController.h"

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4

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
      //memcpy(ImgBuffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
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
       
    if (emDemo->emScanDevice(true) > EM_STATUS_SUCCESS) 
    {
        if(EM_STATUS_SUCCESS == emDemo->emOpenDevice(m_Device_1, 0, MSQ_KEY, true, false))
        {
    		emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
    		printf("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		emDemo->emSetOutputOnceOrMulti(0, 0);
    		
            while(1)
            {
    		    emDemo->emRegisterImageCallback(0, (void*)NULL, OnTestCallBackFun);
			    printf("10 seconds imaging testing, more than 20 times can be used normally,less than 20 please contact:*****\n");
    		    emDemo->emDevStart(0);
    		    usleep(10 * 1000 * 1000);
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
