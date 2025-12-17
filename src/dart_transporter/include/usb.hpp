#pragma once


#include "sdk/include/usbcdc_transporter.hpp"

#define RMOS_RECEIVE_ID 0x1
#define RMOS_SEND_ID 0x2

namespace transporter {


#pragma pack(push, 1)


typedef struct {
  // 包头
  uint8_t _SOF;
  uint8_t ID;
   float pitch;
   float yaw;
   bool is_detectored;
  // 包尾
  uint8_t _EOF;
} SendPackage;

#pragma pack(pop)

}  // namespace transporter

