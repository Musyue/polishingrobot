#ifndef CAN_APPLICATION_H
#define CAN_APPLICATION_H

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "can_drive_pkg/ICANCmd.h"

namespace mobile_base {

class CanApplication {
 public:
  CanApplication();

  virtual ~CanApplication();

  /**
   * @brief Close CAN device
   */
  void CloseCAN();

  /**
   * @brief Open, initialize and start the can device
   * @param file_address The path of configurationfiles
   */
  void ActivateCAN(const std::string& file_address);
 PCAN_DataFrame GetVciObject(const int& obj_num, const uint& initial_id);
  void SendCommand(PCAN_DataFrame obj, const uint& len,
                   const bool& single_frame = false);
  void GetData(PCAN_DataFrame  obj, const int& obj_len);
  int GetBufferLength() {
    return (int)CAN_GetReceiveCount(dwDeviceHandle, can_index_);
  }
  inline void ClearBuffer() {
    CAN_ClearReceiveBuffer(dwDeviceHandle, can_index_);
  }

 private:
  void LoadConfig(const std::string& file_address);
  void CanDiagnostic(const std::string& description, const int& state);

  int device_type_;
  int device_index_;
  DWORD dwDeviceHandle;
  int can_index_;
  int wait_time_;

  int frame_len_;
};  // class CanApplication

}  // namespace mobile_base

#endif
