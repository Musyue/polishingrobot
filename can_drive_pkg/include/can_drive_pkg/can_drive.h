#ifndef _CAN_DRIVE_H
#define _CAN_DRIVE_H
#include "can_drive_pkg/ICANCmd.h"
class can_communication
{
public:
        can_communication();
        ~can_communication();
public:
        unsigned int Can_Open();
        bool Can_Close();
        bool Can_Channel_Start();
        bool Can_Channel_Stop();
        bool Can_Get_Device_Info();
        unsigned long Can_Channel_Send(UINT Send_ID,BYTE Data_length,BYTE SendMessage[]);
        unsigned long Can_Channel_Receive( );
        bool Can_Get_Error_Info();
        CAN_DataFrame Cdf_i[2500];

private:
        DWORD dwDeviceHandle;
        CAN_InitConfig Cic;
        CAN_DeviceInformation Cdi;
        CAN_DataFrame Cdf_o[1];
        CAN_ErrorInformation err;
};

#endif
