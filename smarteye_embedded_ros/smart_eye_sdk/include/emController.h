/*****************************************************************************
*  emPmp Basic tool library                                                  *
*  Copyright (C) 2 0:14 Kean.Ren  rh_king@163.com.                           *
*                                                                            *
*  This file is part of OST.                                                 *
*                                                                            *
*  This program is free software; you can redistribute it and/or modify      *
*  it under the terms of the GNU General Public License version 3 as         *
*  published by the Free Software Foundation.                                *
*                                                                            *
*  You should have received a copy of the GNU General Public License         *
*  along with OST. If not, see <http://www.gnu.org/licenses/>.               *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*  @file     emController                                                    *
*  @brief    实现对设备的控制                                                *
*  Details.                                                                  *
*                                                                            *
*  @author   Kean. Ren                                                       *
*  @email    rh_king@163.com                                                 *
*  @version  1. 0:. 0:.1(版本号)                                             *
*  @date     rh_king@163.com                                                 *
*  @license  GNU General Public License (GPL)                                *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2 0:2 0:/ 0:1/24 | 1. 0:. 1:.1   | Kean. Ren        | Create file         *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#ifndef EMBEDDED_CONTROL_H
#define EMBEDDED_CONTROL_H

#include "emCommon_type.h"

class UdpTransf;
class emControllerExd;

/**
* @brief emController
* declare control the embedded device functions
*/
class emController
{
public:
	emSMARTEYE_API emController();
	emSMARTEYE_API ~emController();

	DeviceInfoScan *emGuiDevScanInfo[MAX_IP_CNT];
	bool m_emDevScan;
	/** 
    * @brief Scan Devices
    * @param quickCon
    *   true : quick finish scan device
    *   false : need other functions like: emInitLibPara,emSetOutputResultsType,emSwitchRunMode.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
	emSMARTEYE_API int emScanDevice(bool quickCon);

    /**
    * @brief Change Devices ip or to make sure ip
    * @param  ip
    *   if ip addr is not equ to orignal,the device ip will changed,and system will be reboot.
    * @param  ip_index
    *   ip index in emGuiDevScanInfo[],This array developer must pay attention and be very useful.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emDevChangeIp(char* ip, int ip_index);

    /**
    * @brief Open Device
    * @param  hDevice
    *   device handle used for init with class emDeviceController,One correspondence with the index value for operating device.
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0:,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param  msg_id
    *   used for linux,if only one device just use MSQ_KEY is ok.
    * @param  quickRun
    *   Quickly turn on the device switch. If true, it will automatically call other functions related to opening the device,
    *   otherwise you need to open it manually, and these functions are required.
    * @param  DataType
    *   The type of output data, that is, 3d data or 2d data
    *   @arg ture:  3d
    *   @arg false: 2d
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emOpenDevice(EM_DEV_HANDLE& hDevice, int nDeviceIndex, int msg_id, bool quickRun = false, bool isPointCloud = false);

    /**
    * @brief Set ROI area
    * @param  nDeviceIndex from 0
    *           startX from 1
    *           startY from 1
    *           width
    *           height
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetRoiValue(const int nDeviceIndex, const int startX, const int startY, const int width, const int height);

    /**
    * @brief Set ROI z-axis value
    * @param  nDeviceIndex from 0
    *           zMin from 1
    *           zMax from 1
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetzAxisValueOfPiontcloud(const int nDeviceIndex, const int zMin, const int zMax);

    /**
    * @brief Close Device
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0:,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API void emCloseDevice(const int nDeviceIndex =  0);

    /**
    * @brief init Device net parameter
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0:,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int  emInitLibPara(const int nDeviceIndex =  0);

    /**
    * @brief register user callback function
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param  pUserParam
    *   user data
    * @param callBackFuncDefinedByUser
    *   user callback functions,attention CallBackFuncDefineByAP which define int emCommon_type.h
    *
    *   pUserParam
    */
    emSMARTEYE_API void* emRegisterImageCallback(const int nDeviceIndex, void *pUserParam, CallBackFuncDefineByAP callBackFuncDefinedByUser);

    /**
    * @brief Start Device
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emDevStart(const int nDeviceIndex =  0);

    /**
    * @brief Stop Device
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emDevStop(const int nDeviceIndex =  0);

    /**
    * @brief Set Device output data type
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param isArmPlatform
    *   @arg 0:parallex
    *   @arg 1:pointCloud
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetOutputCalcPlatform(const int nDeviceIndex, int isArmPlatform);

    /**
    * @brief Set Device run mode
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param sw_mode
    *   @arg 2:idle
    *   @arg 1:3d
    *   @arg 0:2d
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSwitchRunMode(const int nDeviceIndex, int sw_mode);
////////////////////////
    /**
    * @brief Set Device trigger source
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param sw_mode
    *   @arg 0:inside,device inside means projector
    *   @arg 1:outside,other type such as robot,gpio and so on
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetTriggerSource(const int nDeviceIndex, int triggerSource);

    /**
    * @brief Set Device output reselut mode
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param outputMode
    *   @arg 0:run once a time
    *   @arg 1:continu
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetOutputOnceOrMulti(const int nDeviceIndex, const int onecOrmulti);

    /**
    * @brief set device run needed parameters
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param runMode
    *  same as emSwitchRunMode();
    * @param outputType
    *  same as emSetOutputResultsType();
    * @param outputMode
    *  same as emSetOutputResultsMode();
    * @param triggerType
    *  same as emSetTriggerSource();
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetRunParam(const int nDeviceIndex, const int runMode, const int outputType, const int outputMode, const int triggerType);

    /**
    * @brief Set/Get Device exposure time
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param extime
    *    0:get device exposure time
    *   xx:set device exposure time,must more then 10us
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSetExposureTime(const int nDeviceIndex, const uint32_t extime, const int typeDev = 0);
    emSMARTEYE_API int emGetExposureTime(const int nDeviceIndex, uint32_t &extime);

    /**
    * @brief Get Device Scan status
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval ture: scan over
    * @retval fail: not scan
    */
    emSMARTEYE_API bool emGetDeviceScan();

    /**
    * @brief Read embedded device current info
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param allFrameCnt
    *   Embedded device running calculations times
    * @param powerOnTime
    *   Embedded device running time form power on
    * @param frameRate
    *   Embedded device running fps
    * @param inSideTemp
    *   Embedded device temperature
    * @param tempStatu
    *   Embedded device temperature key word
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emGetSysStatus(const int nDeviceIndex, uint64_t *allFrameCnt, uint64_t *powerOnTime, int32_t *frameRate, int32_t *inSideTemp, int32_t *tempStatu);
    emSMARTEYE_API int emGetSysStatusEx(const int nDeviceIndex, SysInfo& sysStatu);
    /**
    * @brief get embedded device info
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param ip
    *   current connect device ip address
    * @param mac
    *   current connect device mac id
    * @param sn
    *   current connect device sn number
    * @param version
    *   current connect device version info
    * @param name
    *   current connect device name
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emGetSysSensorInfo(const int nDeviceIndex, char* ip, char *mac, char *sn, char *version, char *name);

    /**
    * @brief read embedded device configure ini file
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param loadputfilePath
    *   Indicate configure ini file put local placement path
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emLoadConfigIniFile(const int nDeviceIndex, const char* loadputfilePath);

    /**
    * @brief recovery foramt mode
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emRecoveryConfigIniFile(const int nDeviceIndex =  0);

    /**
    * @brief load local configini to embedded device
    * @param  nDeviceIndex
    *   device index the same index with ip_index in emGuiDevScanInfo[],begin with  0,when more then one device,1 index is sencod device
    *   and so on.this param is can be used for operation opened device.
    * @param filepath
    *   Indicate configure ini file placement path
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int emSyncLocalConfigIniFile(const int nDeviceIndex, char* fileName);

    /**
    * @brief Get q matrix
    * @param  QData
    *   struct Qmax
    *    {
    *        double b;
    *        double f;
    *        double X 0;
    *        double Y 0;
    *    }
    * @param qFilePath
    *   Indicate q matrix placement path
    *
    * @retval -1: fail
    * @retval  0: succeed
    */
    emSMARTEYE_API int upQmaxData(Qmax* QData, const char* qFilePath);

    /**
    * @brief output pointcloud data
    * @param  parallexBuf
    *   parallex data,which form embedded device,Four bytes represent a value,Lower 12 digits represent decimals.
    * @param pointCloud
    *   XYZRGB(X,Y,Z, 0, 0, 0)
    *
    */
    emSMARTEYE_API int emExchangeParallaxToPointCloud(uint8_t* parallexBuf, uint8_t *grayImg, PointCloud_EM_Ptr& pointCloud);
    emSMARTEYE_API int emExchangeParallaxToPointCloudEx(uint8_t* parallexBuf, uint8_t *grayImg, PointCloud_EM_Ptr& pointCloud);
    emSMARTEYE_API int emExchangeParallaxToPointCloudMonocular(uint8_t* parallexBuf, uint8_t *grayImg, PointCloud_EM_Ptr& pointCloud);
private:

    void timeoutOperation();
    //int  connectDevice(unsigned int deviceIndex);
    //void disConnectDevice();

    //void upstrDeviceListsCtl(const DeviceInfoScan* src, DeviceListsCtl* dest, int cnt);


    UdpTransf* udpPriClint;
    emControllerExd* exd;

#if defined(_WIN32)
    HANDLE thread_mutex;
#else
    pthread_mutex_t thread_mutex;
#endif
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
#endif // EMBEDDED_CONTROL_H


