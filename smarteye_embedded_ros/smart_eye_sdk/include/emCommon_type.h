#ifndef _EMCOMMON_TYPE_H
#define _EMCOMMON_TYPE_H

//------------------------------------------------------------------------------
//  操作系统平台定义
//------------------------------------------------------------------------------
#include <stddef.h>
#ifdef WIN32
    #ifndef _WIN32
        #define _WIN32
    #endif
#endif

#ifdef _WIN32
	#include <windows.h>
	#include <stdio.h>
    #include <objbase.h>
    #include <process.h>
	#include <Winsock2.h>
	#include "WS2tcpip.h"

	#ifdef emSMARTEYE_STATIC
        #define DLL_API
    #else
		#ifdef DLL_EXPORTS
			#define emSMARTEYE_API   __declspec(dllexport)
		#else
			#define emSMARTEYE_API   __declspec(dllimport)
		#endif// DLL_EXPORTS
	#endif// emSMARTEYE_STATIC

    #define EM_STDC __stdcall
    #define EM_CDEC __cdecl

    #if defined(__cplusplus)
        #define EM_EXTC extern "C"
    #else
        #define EM_EXTC
    #endif
#else
    // remove the None #define conflicting with GenApi
	#include "pthread.h"
	//// it means the compiler is GCC version 4.0 or later
    #if __GNUC__ >= 4
		#ifdef DLL_EXPORTS
//			#warning "===== dynamic library ====="
			#define emSMARTEYE_API   __attribute__((visibility("default")))
			#define emSMARTEYE_API_LOCAL __attribute__((visibility("hidden")))
		#else
			#warning "===== static library ====="
			#define emSMARTEYE_API
			#define emSMARTEYE_API_LOCAL
		#endif

        #ifdef __cplusplus
            #define EM_EXTC extern "C"
        #else
            #define EM_EXTC
        #endif
    #else
   pthread     #error "##### requires gcc version >= 4.0 #####"
    #endif
	#define CALLSPEC
#endif

//////////////////////////////////////////////////////////////////////////
//  类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//  VS2010之前的版本中都不包含此文件,所以在此需要重定义
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
    #ifndef _STDINT_H
        #ifdef _MSC_VER // Microsoft compiler
            #if _MSC_VER < 1600
                typedef __int8            int8_t;
                typedef __int16           int16_t;
                typedef __int32           int32_t;
                typedef __int64           int64_t;
                typedef unsigned __int8   uint8_t;
                typedef unsigned __int16  uint16_t;
                typedef unsigned __int32  uint32_t;
                typedef unsigned __int64  uint64_t;
            #else
                // In Visual Studio 2010 is stdint.h already included
                #include <stdint.h>
            #endif
        #else
            // Not a Microsoft compiler
            #include <stdint.h>
        #endif
    #endif
#else
    // Linux
    #include <stdint.h>
#endif

//------------------------------------------------------------------------------
//  算法模式定义 单目多曝光或者双目单曝光算法
//------------------------------------------------------------------------------
//#define XILINX_ZYNQMP_MONOCULAR_ALGRITHIM   1

/************************************************************************************/
/************************************************************************************/
//public header files
#include <iostream>
#include <vector>
#include <string.h>
#include <memory.h>
#include <exception>

#define SETALIGN	sizeof(int)

#ifdef _WIN32
	#define GNUC_ALIGN(n)
#else
	#ifdef __GNUC__
		#define GNUC_ALIGN(n) __attribute__((aligned(n)))
	#endif
#endif

#ifdef _WIN32
	using namespace std;
#endif

/***********************************************************************************/
/***********************************************************************************/
#define    EF(errflags)     errflags
#define    EG(errsuggest)   errsuggest
#define    POPULATEERRORINDICATIONS(errno, errflags, errsuggest) \
    do {                                        \
        char *ef = EF(errflags);                \
        char *eg = EG(errsuggest);              \
        apiError.errCode = errno;               \
        apiError.errFlags = ef;                 \
        apiError.suggestedAction = eg;          \
        }while(0)

#define POEI(errno, errflags, errsuggest)   POPULATEERRORINDICATIONS(errno, errflags, errsuggest)

/*************************************************************************************
 * cmd
 * ***********************************************************************************/
 //pcl
#define GUI_CMD_CONTROL_SET_PARALLAX_ON     "SetOutputParallaxOn" //设置输出计算结果类型 [cmdx]x:0 视差 x:1 点云

//calibrations
#define GUI_CMD_CALIB_TO_CALIB_MODE    		"CmdToCalibModal" 	//切换至在线标定模式
#define GUI_CMD_CALIB_ACTIVE_GETIMAGE     	"CmdStartGetImg"	//开始获取图片
#define GUI_CMD_CALIB_ACTIVE_CALC	     	"CmdStartCalC"		//开始计算
#define GUI_CMD_CALIB_SET_BOARDWIDTH	    "SetBoardWidth"		//设置标定板宽[cmdx]
#define GUI_CMD_CALIB_SET_BOARDHIGHT	    "SetBoardHight"		//设置标定板高[cmdx]
#define GUI_CMD_CALIB_SET_PATTERNNUM	    "SetPatternNum"		//设置标定照片张数[cmdx]
#define GUI_CMD_CALIB_SET_SQUARESIZE	    "SetSquareSize"		//设置方格大小[cmdx]
#define GUI_CMD_CALIB_ACTIVE_SHOWRESULT 	"CmdShowResult"		//输出计算结果
#define GUI_CMD_CALIB_SET_EXPOSURETIME      "SetExposureTime"	//设置曝光时间[cmdx]
#define GUI_CMD_CALIB_TO_OFFLINE_MODE		"CmdToCalOffLineModal" //切换离线标定模式

//camera
#define GUI_CMD_CAMERA_GET_MINEXPOSURE      "CmdGetMinExposure"	//获取最小曝光时间
#define GUI_CMD_CAMERA_ACTIVE_TRIGGERONCE   "CmdTriggerOnce"	//触发一次 一组patterns
#define GUI_CMD_CAMERA_SET_TRIGGERTIMES     "SetTriggerTimes"	//触发次数[cmdx]
#define GUI_CMD_CAMERA_SET_TRIGGEREACHTIME  "SetTriggerEachTime"//触发间隔[cmdx]需要大于min
#define GUI_CMD_CAMERA_SET_TRIGGER_MODE    	"SetTriggerMode"	//触发模式 [cmdx]0:hardware 1:software

//projector
#define GUI_CMD_PROJECTOR_ACTIVE_CONNECT        	"CmdConnectPorjector" //链接投影
#define GUI_CMD_PROJECTOR_ACTIVE_RESET       		"CmdReConnectPorjector"//重连投影
#define GUI_CMD_PROJECTOR_ACTIVE_VALIDATE    		"CmdValidate"		//投影检测是否有效
#define GUI_CMD_PROJECTOR_ACTIVE_STOP    			"CmdProjectorStop"	//停止投影
#define GUI_CMD_PROJECTOR_ACTIVE_START    			"CmdProjectorStart"	//打开投影
#define GUI_CMD_PROJECTOR_ACTIVE_PAUSE    			"CmdProjectorPause"	//暂停投影
#define GUI_CMD_PROJECTOR_TO_UPLOAD_MODE			"CmdProjectorUpLoad"//烧写patterns

//net
#define GUI_CMD_NET_SET_RESLUTIONRADIA       		"SetReslutionRadia" //设置图片分辨率[cmdx]

//run
#define GUI_CMD_CONTROL_SET_TRIGGER_SOURCE    			"SetSysTriggerSource"	//触发模式 [cmdx]0:outside 1:inside
#define GUI_CMD_CONTROL_SET_RUNLEVEL					"CmdRunLevel"	//设置运行等级
#define GUI_CMD_CONTROL_ACTIVE_START					"CmdStartRun"	//开始
#define GUI_CMD_CONTROL_ACTIVE_STOP						"CmdStopRun"	//停止
#define GUI_CMD_CONTROL_ACTIVE_CAL						"CmdStartCal"	//FPGA计算
#define GUI_CMD_CONTROL_ACTIVE_SRC						"CmdStartSrc"	//原图
#define GUI_CMD_CONTROL_SET_CAL_TYPE					"SetCalType"	//连续模式还是单次模式[cmdx]0:单次，1：连续
#define GUI_CMD_CONTROL_SET_SRC_LEFT_ADD        		"SetSrcLeftAdd"	//原图左边显示加[cmdx]
#define GUI_CMD_CONTROL_SET_SRC_RIGHT_ADD       		"SetSrcRightAdd"//原图右边显示加[cmdx]
#define GUI_CMD_CONTROL_SET_CAL_M_ADD					"SetCalMadd"	//FPGA计算结果通道加[cmdx]
#define GUI_CMD_CONTROL_ACTIVE_JUMPROUTER          		"CmdJumpRouter"	//跳过路由配置即发现
#define GUI_CMD_CONTROL_GET_RUNTIME						"GetRunTime"	//获取系统运行时间
#define GUI_CMD_CONTROL_GET_FPGATIME					"GetFpgaTime"	//获取系统成像次数
#define GUI_CMD_CONTROL_GET_TEMPRATURE                  "GetTemprature" //获取温度
#define GUI_CMD_CONTROL_GET_TEMP_STATUS					"GetTempStatus" //获取温度状态
#define GUI_CMD_CONTROL_GET_FRAMERATE                   "GetFrameRate"  //获得帧率
#define GUI_CMD_CONTROL_GET_SYSTEM_MODE                 "GetSystemMode"  //获得系统状态
#define GUI_CMD_CONTROL_ACTIVE_RECOV_CONF				"CmdRecovPara"	//配置文件恢复出厂设置
#define GUI_CMD_CONTROL_ACTIVE_LOAD_CONF				"CmdLoadPara"	//客户端读取配置文件
#define GUI_CMD_CONTROL_ACTIVE_SYNC_CONF				"CmdSyncPara"	//同步配置文件，加载用户配置文件

#define GUI_CMD_CONTROL_GET_SYSINFO						"GetSysInfo"	//获取系统信息
#define GUI_CMD_CONTROL_SET_ROI_PERPOTIES               "SetRoiPerpoties"   //set roi value
#define GUI_CMD_CONTROL_SET_Z_AXIS                      "SetZaxisValue" //pointcloud' z-axis value

#define GUI_CMD_CONTROL_TO_2DCAMERA_MODE 				"CmdProCamerLookMode" //进入2D模式
#define GUI_CMD_CONTROL_TO_NORMAL_MODE         			"CmdRunNormalMode"//进入正常模式
#define GUI_CMD_CONTROL_TO_IDLE_MODE         			"CmdRunIdleMode"//进入空闲模式

#define GUI_CMD_CONTROL_EXIT							"CmdExitApp"	//退出APP
#define GUI_CMD_CONTROL_TCP_TEST                        "TCP"

/****************************************************************************************
 * OTHER DEFINE
 * **************************************************************************************/
#define MAX_MSG_LEN                                     2048
#define MSQ_KEY                                         1005

#define THREADCNT                                       5

#define STR_LENGTH										8
#define MAX_IP_CNT                                      32
#define MAX_IP_LENGTH									64

#define NUM_NETWOR_COMMAND_FIELDS                       6

#define SocketAddressSize  sizeof(struct sockaddr_in)
#define PACAGESIZE (4096)

//--------------------data-struct------------------------------
//------------------------------------------------------------------------------
//  错误码定义
//------------------------------------------------------------------------------
enum EM_STATUS_LIST {
    EM_STATUS_SUCCESS                       =  0,           ///<success>
    EM_STATUS_ERROR                         = -1,           ///<不期望发生的未明确指明的内部错误>
    EM_STATUS_INIT_HANDLE_ERROR             = -2,           ///<获取设置句柄失败>
    EM_STATUS_DEVICE_OPENED_AGINA           = -3,           ///<设备已打开，再次打开失败>
    EM_STATUS_INIT_NEWORK_ERROR             = -4,           ///<初始化网络链接失败>
    EM_STATUS_INVALID_PARAMETER             = -5,           ///<无效参数,一般是指针为NULL或输入的IP等参数格式无效>
    EM_STATUS_INVALID_HANDLE                = -6,           ///<无效句柄>
    EM_STATUS_INVALID_CALL                  = -7,           ///<无效的接口调用,专指软件接口逻辑错误>
    EM_STATUS_INVALID_ACCESS                = -8,           ///<功能当前不可访问或设备访问模式错误>
    EM_STATUS_NEED_MORE_BUFFER              = -9,           ///<用户申请的buffer不足:读操作时用户输入buffersize小于实际需要>
    EM_STATUS_ERROR_TYPE                    = -10,          ///<用户使用的FeatureID类型错误比如整型接口使用了浮点型的功能码>
    EM_STATUS_OUT_OF_RANGE					= -11,			///<用户写入的值越界>
    EM_STATUS_NOT_INIT_API					= -12,			///<没有调用初始化接口>
    EM_STATUS_INVALID_SOCKED				= -13,          ///<SOCKED ERROR>
    EM_STATUS_TIMEOUT                       = -14,          ///<ACCESS TIMEOUT>
    EM_STATUS_NOFILE_ERROR                  = -15,          ///<FILE OPERATION ERROR>
    EM_STATUS_SCAMDEVICE_ERROR              = -16,          ///<SCAN DEVICE FAILED>
    EM_STATUS_CHANGEDEVIP_ERROR             = -17,          ///<MODEFY REMOTE IP ERROR>
    EM_STATUS_NOSCANDEVICE_ERROR			= -18,			///<NO DEVICE SCANED>
    EM_STATUS_OPENPROJECOR_ERROR            = -19,          ///<OPEN PROJECTOR ERROR>
	EM_STATUS_NOT_IMPLEMENTED				= -20,          ///<当前不支持的功能>
	EM_STATUS_INIT_DEVICE_ERROR				= -21,			///<DEVICE INIT FAILED>

	EM_STATUS_CONNECT_TCP_ERROR				=-100,			///<TCP CONNECT ERROR>
	EM_STATUS_CONNECT_UDP_ERROR				=-101,			///<UDP CONNECT ERROR>
	EM_STATUS_UDP_BINDUP_ERROR				=-102,			///<UDP BIND FAILED>
	EM_STATUS_TCP_DISCONNECT				=-102,			///<TCP LOST CONNECT>

	EM_STATUS_QUEUE_FULL_ERROR				=-200,			///<QUEUE FULL ERROR>
	EM_STATUS_QUEUE_EMPTY_ERROR				=-201,			///<QUEUE EMPTY ERROR>
	EM_STATUS_QUEUE_NOINIT_ERROR			=-202,			///<QUEUE NO INITITUTION ERROR>

	EM_STATUS_BUFRESIZE_ERROR				=-300,			///<RECV BUFFER RESIZE ERROR>
	EM_STATUS_SENDMSG_THREAD_ERROR			=-301,			///<
	EM_STATUS_THREAD_RUN_ERROR				=-302,			///<EROOR IN THREAD>
	EM_STATUS_CMD_ERROR						=-303,			///<SEND CMD ERROR>
	EM_STATUS_THREAD_PARE_ERROR				=-304,			///<SEND CMD ERROR>
};
//typedef enum EM_STATUS_LIST EM_STATUS_LIST;
typedef int32_t EM_STATUS;

//typedef enum EM_THREAD_ID_NUM
enum EM_THREAD_ID_NUM{
	EM_ENUM_ID_CALLBACK_USER    = 0,               	///<数据返回用户线程
    EM_ENUM_ID_TCP_SEND         = 1,               	///<TCP发送回调线程
    EM_ENUM_ID_TCP_RECV    		= 2,				///<TCP接受数据回调函数
    EM_ENUM_ID_UDP_RECV         = 3,               	///<UDP发送数据线程
    EM_ENUM_ID_UDP_DECODE       = 4,               	///<UDP发送数据线程
	EM_ENUM_ID_THREAD_CNT_MAX
};
//typedef enum EM_THREAD_ID_NUM EM_THREAD_ID_NUM;

enum EM_NET_TYPE{
    EM_NET_TCP = 0,
    EM_NET_UDP = 1,
	EM_NET_ALL = 2
};

enum EM_NET_TCP{
	EM_TCP_CMD = 0,
	EM_TCP_RECV = 1
};

enum EM_STATU_SW{
    EM_OFF = 0,
    EM_ON  = 1
};

typedef enum EM_STATU_SW EM_CALC_MODE ;
typedef enum EM_STATU_SW EM_3D_RUN_TYPE;

enum EM_RUN_TYPE {
	ONCETIME = 0,
	CONTINUE = 1
};

enum EM_TRIGGER_TYPE {
	INSIDE = 0,
	OUTSIDE = 1
};

enum EM_OUTPUT_TYPE {
	PARALLAX = 0,
	PIONTCLOUD = 1
};

enum EM_RUN_WORKMODE{       
    EM_2D_ORGLOOK  = 0,
    EM_NORMAL_3D   = 1,
    EM_IDLE        = 2,

};
typedef enum EM_RUN_WORKMODE EM_RUN_WORKMODE;

enum EM_RUN_MODE{
    EM_RUN_2D_MODE = 0,
    EM_RUN_3D_MODE = 1
};
typedef enum EM_RUN_MODE EM_RUN_MODE;

enum EM_CLC_MODE {
	EM_CLC_PC = 0,
	EM_CLC_ARM = 1
};
//------------------------------------------------------------------------------
//  标准C API功能函数定义
//------------------------------------------------------------------------------
#define EM_API EM_EXTC EM_STATUS EM_STDC

struct initError{
    uint8_t iniConfigFlag: 1;
    uint8_t drvWriteFlag : 1;
    uint8_t drvReadFlag  : 1;
    uint8_t reMapLoadFlag: 1;
    uint8_t projTimeFlag : 1;
    uint8_t cameraFlag   : 1;
    uint8_t xadcFlag     : 1;
    uint8_t mcp3021Flag  : 1;
}GNUC_ALIGN(SETALIGN);
typedef struct initError    initDeviceError;

struct cmdParam {
    char cmd_head[10];
    char cmd_type[6];
    char cmd_ip[MAX_IP_CNT];
    char cmd_port[STR_LENGTH];
    char cmd_data[MAX_MSG_LEN];
    char cmd_tril[STR_LENGTH];
}GNUC_ALIGN(SETALIGN);
typedef struct cmdParam	 cmdParam;

struct userCmd {
    int inum;
    void* pdata;
}GNUC_ALIGN(SETALIGN);
typedef struct userCmd	 userCmd;
/******************
 * define threadparam
 ******************/
struct ThreadParam
{
    char name[MAX_IP_LENGTH] ;
#ifndef _WIN32
    pthread_t 		id   ;	// id
    pthread_attr_t 	attr ;	// attr
#else
	HANDLE thrHand;
	unsigned int nThreadID;
#endif
    int begin 	;
    int running ;
    int end 	;

    int err_code    ;
    char *err_flag  ;

}GNUC_ALIGN(SETALIGN);
typedef struct ThreadParam	 ThreadParam;
/* @struct: packeg header
 *
 * */
struct pak_header
{
    char    ID[STR_LENGTH] ;  //数据身份识别 目前定义为：imdata 即图片数据  STR_LENGTH = 8
    int        width ;        //图像宽度
    int        height ;       //图像高度
    int        plen ;         //每一次发送数据包长度，最后一包的长度会与前面不同
    int        index ;        //当前包数据在整个数据中的位置，用于解决数据包错位
    int        arg1 ;         //当前被赋值为当前数据包在整个数据包中的index,即韦博士提到的
    int        arg2 ;         //当前被赋值为当前数据包在整个数据包中的index的ASCII码，用于抓包查看
    int        m ;            // 当前帧数据最后一包标识位，用于检测数据是否为最后一包
    int        type;          //标识数据种类 0:左相机图像 1:右相机图像 2:标定结果图像 3:FPGA计算结果图像
    int        recv1;          //保留
    int        recv2;         //保留
}GNUC_ALIGN(SETALIGN);
typedef struct pak_header	 pak_header;

struct Msg
{
    long msgType;
    char msgText[MAX_MSG_LEN];

}GNUC_ALIGN(SETALIGN);
typedef struct Msg	 Msg;

struct Qmax
{
    double b;
    double f;
    double X0;
    double Y0;
}GNUC_ALIGN(SETALIGN);
typedef struct Qmax	 Qmax;

struct RICB
{
	float rate;	
	int	  arg;
	void* argc;
}GNUC_ALIGN(SETALIGN);
typedef struct RICB RICB;

struct ROIPROPOTIES
{
	int startX;
	int	width;
	int startY;
	int height;
	int pixSum;
}GNUC_ALIGN(SETALIGN);
typedef struct ROIPROPOTIES strROI;

//UDP_PROB
struct ScanProb
{
	int				Video_Port;	//VIDEO PORT
	int				CMD_Port;	//CMD PORT
	int				Ctrl_Port;	//CTRL PORT
	int				Res_Port;	//RES PORT
	char			IP[MAX_IP_CNT];
	char			MAC[MAX_IP_CNT];
	char			Types[MAX_IP_CNT];
	char			Name[MAX_IP_LENGTH];
	char			SN[MAX_IP_LENGTH];
	char			Version[MAX_IP_LENGTH*2];
}GNUC_ALIGN(SETALIGN);
typedef struct ScanProb	 ScanProb;

union uScanDevInfo
{
	ScanProb scanDevInfoAll;
	char udevInfo[512];
}GNUC_ALIGN(SETALIGN);
typedef union uScanDevInfo	 uScanDevInfo;
//LOCAL_PROBE_
struct LOCAL_uScanDevInfo
{
	ScanProb ScanDevInfo[MAX_IP_CNT];
    int      localip_index;
    int      localip_num;
}GNUC_ALIGN(SETALIGN);
typedef struct LOCAL_uScanDevInfo	 LOCAL_uScanDevInfo;

struct DeviceInfoScan
{
    char			IP[MAX_IP_CNT];
    char			MAC[MAX_IP_CNT];
    char			SN[MAX_IP_LENGTH];
    char			Version[MAX_IP_LENGTH*2];
    char			Name[MAX_IP_LENGTH];
    char            lIP[MAX_IP_LENGTH];
}GNUC_ALIGN(SETALIGN);
typedef struct DeviceInfoScan	DeviceInfoScan;
typedef DeviceInfoScan*  pDeviceInfoScan;

struct DeviceListsCtl
{
	void* devHandle;
    char ScanIP_IP[MAX_IP_LENGTH];
    char LocalIP_IP[MAX_IP_LENGTH];
    char ScanIP_Mac[MAX_IP_CNT];
    bool vailUse;
    bool vailNet;
}GNUC_ALIGN(SETALIGN);
typedef struct DeviceListsCtl	 DeviceListsCtl;

struct SysInfo
{
	int32_t  gsystemMode;
	uint32_t gframeRate;
	uint32_t ginSideTemp;
	uint32_t gtempStatu;
	uint32_t gpowerOnTime;
	uint32_t gallFrameCnt;
	uint32_t gexpTime;
	uint32_t ginitDeviceFlag;
}GNUC_ALIGN(SETALIGN);
typedef struct SysInfo	 SysInfo;

typedef void* EM_DEV_HANDLE;				///<设备句柄，通过EMOpenDevice获取，通过此句柄进行控制与采集>
typedef void* EM_EVENT_CALLBACK_HANDLE;		///<设备事件回调句柄，注册设备相关事件回调函数，比如设备掉线回调函数>

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////POINTCLOUD-STR///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
struct PointXYZRGB
{
    union {
        float data[4];
        struct {
            float x;
            float y;
            float z;
        };
    };

    union {
        union {
            struct {
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;
            };
            float rgb;
        };
        uint32_t rgba;
    };

    inline PointXYZRGB ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
    }

    inline PointXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = _r;
      g = _g;
      b = _b;
      a = 255;
    }
};


/**
 * @brief       PointCloud is used for storing collections of 3D points.
 */
#include <vector>
#include <memory>

template <typename PointT>
class PointCloud
{
public:
    PointCloud() : points(), width(0), height(0) {}
    PointCloud(PointCloud<PointT> &pc) : points (), width (0), height (0)
    {
        *this = pc;
    }

    virtual ~PointCloud() {}

    inline const PointT&
    at (int column, int row) const
    {
      if (this->height > 1)
        return (points.at (row * this->width + column));
      else
        throw std::exception("Can't use 2D indexing with a unorganized point cloud");
    }

    inline PointT&
    at(int column, int row)
    {
        if (this->height > 1)
            return (points.at(row * this->width + column));
        else
            throw std::exception("Can't use 2D indexing with a unorganized point cloud");
    }

    inline const PointT&
    operator () (size_t column, size_t row) const
    {
        return (points[row * this->width + column]);
    }

    inline PointT&
    operator () (size_t column, size_t row)
    {
        return (points[row * this->width + column]);
    }

    //capacity
    inline size_t size () const { return (points.size ()); }
    inline void reserve (size_t n) { points.reserve (n); }
    inline bool empty () const { return points.empty (); }

    inline void resize (size_t n)
    {
        points.resize (n);
        if (width * height != n)
        {
          width = static_cast<uint32_t> (n);
          height = 1;
        }
    }

    //element access
    inline const PointT& operator[] (size_t n) const { return (points[n]); }
    inline PointT& operator[] (size_t n) { return (points[n]); }
    inline const PointT& at (size_t n) const { return (points.at (n)); }
    inline PointT& at (size_t n) { return (points.at (n)); }
    inline const PointT& front () const { return (points.front ()); }
    inline PointT& front () { return (points.front ()); }
    inline const PointT& back () const { return (points.back ()); }
    inline PointT& back () { return (points.back ()); }

    inline void push_back (const PointT& pt)
    {
        points.push_back (pt);
        width = static_cast<uint32_t> (points.size ());
        height = 1;
    }

    inline void clear ()
    {
        points.clear ();
        width = 0;
        height = 0;
    }

    /** \brief The point data. */
    std::vector<PointT> points;
    /** \brief The point cloud width (if organized as an image-structure). */
    uint32_t width;
    /** \brief The point cloud height (if organized as an image-structure). */
    uint32_t height;
    typedef std::shared_ptr<PointCloud<PointT> > Ptr;
};

typedef PointXYZRGB PointType;
typedef PointCloud<PointType> PointCloud_EM;
typedef PointCloud_EM::Ptr PointCloud_EM_Ptr;

/********************************
 * define CALLBACK_PARAM
 ********************************/
struct PNP_FRAME_CALLBACK_PARAM
{
    void* pUserParam    ;
//    union {
      void* pImgBuf;
      PointCloud_EM_Ptr pImgBufPointCloud;
      uint8_t *grayImg;
//    };
//    void* pImgBuf ;
    uint8_t statuFrame;	//0:current 2:resendFrame 1:in lost rate
    uint64_t nFrameID   ;
    uint64_t pBufferSize;
    uint32_t *reserved  ;

}GNUC_ALIGN(SETALIGN);
typedef struct PNP_FRAME_CALLBACK_PARAM	 PNP_FRAME_CALLBACK_PARAM;

typedef void (* CallBackFuncDefineByAP)(PNP_FRAME_CALLBACK_PARAM *pFrame);     //usr inter to deal buffer

typedef struct {
    int errCode;
    char *errFlags;
    char *suggestedAction;
}errorNote;

extern errorNote apiError;


#endif // EMBEDDEDAPI_H


