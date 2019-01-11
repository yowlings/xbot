/**
 * @file include/xbot_driver/packets/core_sensors.hpp
 *
 * @brief Imu sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_SENSORS_HPP__
#define XBOT_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../macros.hpp"
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot
{

/*****************************************************************************
** Interface
*****************************************************************************/

class xbot_PUBLIC Sensors : public packet_handler::payloadBase
{
public:
  Sensors() : packet_handler::payloadBase(false, 72) {};

  struct Data {
    unsigned char yaw_platform_degree;
    unsigned char pitch_platform_degree;
    unsigned char sound_status;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;

    float yaw;
    float pitch;
    float roll;
    float q1;
    float q2;
    float q3;
    float q4;
    unsigned char error_status;
    unsigned short timestamp;


  } data;


  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace xbot

#endif /* XBOT_SENSORS_HPP__ */
