#ifndef WAYPOINTMONITOR_H
#define WAYPOINTMONITOR_H

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <sstream>

class WaypointMonitor
{
public:
    WaypointMonitor();

public:
    static void RealTimeWaypointCallback (const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg); //用于获取实时路点回调函数

    static void startup();

private:

    static bool S_WaypointEqual;

    static bool S_IOCurrnetEnable;

    static double S_monitorJointAngle[6];

    static double S_deviation;

    static std::string S_IO_name;
};

#endif // WAYPOINTMONITOR_H
