#include "waypointmonitor.h"

#include <QSettings>

#include <unistd.h>
#include <string.h>
#include <stdio.h>

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

double WaypointMonitor::S_monitorJointAngle[6] = {0};
double WaypointMonitor::S_deviation = 2.0;

bool  WaypointMonitor::S_WaypointEqual = false;
bool  WaypointMonitor::S_IOCurrnetEnable = false;


std::string WaypointMonitor::S_IO_name="";

WaypointMonitor::WaypointMonitor()
{
}

void WaypointMonitor::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;

    for(int i=0;i<6;i++)
    {
        if(fabs( waypoint.jointpos[i]*180.0/3.1415926 - S_monitorJointAngle[i] )>S_deviation)
        {
            S_WaypointEqual = false;
            break;
        }

        if(i==5)
        {
            S_WaypointEqual = true;
        }
    }
}

void WaypointMonitor::startup()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    S_WaypointEqual   = false;
    S_IOCurrnetEnable = false;

    QSettings setting("/root/AuboRobotWorkSpace/waypointMonitor/joint.conf", QSettings::IniFormat);
    S_monitorJointAngle[0] = setting.value("joint/joint1").toDouble();
    S_monitorJointAngle[1] = setting.value("joint/joint2").toDouble();
    S_monitorJointAngle[2] = setting.value("joint/joint3").toDouble();
    S_monitorJointAngle[3] = setting.value("joint/joint4").toDouble();
    S_monitorJointAngle[4] = setting.value("joint/joint5").toDouble();
    S_monitorJointAngle[5] = setting.value("joint/joint6").toDouble();

    S_deviation = setting.value("param/deviation").toDouble();
    S_IO_name   = setting.value("IO/io_name").toString().toStdString();

    //调试输出
    for(int i=0;i<6;i++)
    {
        std::cout<<"joint"<<i<<":"<<S_monitorJointAngle[i]<<std::endl;
    }
    std::cout<<"偏差角度:"<<S_deviation<<std::endl;
    std::cout<<"IO名称:"<<S_IO_name<<std::endl;

    sleep(30);

    while(true)
    {
        std::cout<<"路点监控程序启动."<<std::endl;

        ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");

        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"登录成功."<<std::endl;
        }
        else
        {
            sleep(1);
            std::cerr<<"登录失败."<<std::endl;
            continue;
        }

        //注册实时路点回调函数
        robotService.robotServiceRegisterRealTimeRoadPointCallback(WaypointMonitor::RealTimeWaypointCallback, NULL);

        while(true)
        {
            if(S_WaypointEqual==true && S_IOCurrnetEnable == false)
            {
                std::cout<<"符合设置IO有效的条件"<<std::endl;
                ret = robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, S_IO_name.c_str(), 1);
                if(ret==aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    S_IOCurrnetEnable = true;
                }
                else
                {
                    std::cout<<std::endl<<"设置IO有效失败:ret="<<ret<<std::endl;
                }
            }

            if(S_WaypointEqual==false && S_IOCurrnetEnable == true)
            {
                std::cout<<"符合设置IO无效的条件==="<<std::endl;
                ret = robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, S_IO_name.c_str(), 0);
                if(ret==aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    S_IOCurrnetEnable = false;
                }
                else
                {
                    std::cout<<std::endl<<"设置IO无效失败:ret="<<ret<<std::endl;
                }
            }

            usleep(10);
        }
    }
}
