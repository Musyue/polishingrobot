#include "smarteye_embedded_ros/smarteye_with_viewpoint_and_env_explorition.h"

using smarteye_viewpoint_and_env_explorition::SmarteyeViewpointEnvExp;

SmarteyeViewpointEnvExp::SmarteyeViewpointEnvExp()
{

}
void SmarteyeViewpointEnvExp::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};
void SmarteyeViewpointEnvExp::registerPubSub()
{
    downsample_pub = nh.advertise<sensor_msgs::PointCloud2> ("smarteye_downsample_output", 1);
    aubo_movej_pub = nh.advertise<std_msgs::String> ("/aubo_ros_script/movej", 1); 
    aubo_movet_pub = nh.advertise<std_msgs::String> ("/aubo_ros_script/movet", 1);
    aubo_movel_pub = nh.advertise<std_msgs::String> ("/aubo_ros_script/movel", 1);
    // sub_f64 = nh.subscribe("chatter2to1",1000,&tl1::fCallback, this);
}
void SmarteyeViewpointEnvExp::OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM* pFrame)()
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
int SmarteyeViewpointEnvExp::cmp(const void *pleft, const void *pright){
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

void SmarteyeViewpointEnvExp::vision_opreating_with_smarteye()
{

}