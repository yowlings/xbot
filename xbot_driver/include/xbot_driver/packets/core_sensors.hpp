/**
 * @file include/xbot_driver/packets/core_sensors.hpp
 *
 * @brief Core sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_CORE_SENSORS_HPP__
#define XBOT_CORE_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../macros.hpp"
#include "../lqueue.h"
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot
{

/*****************************************************************************
** Interface
*****************************************************************************/

class xbot_PUBLIC CoreSensors : public packet_handler::payloadBase
{
public:
  CoreSensors() : packet_handler::payloadBase(false, 30),
    queue_front_infrared(25),
    queue_rear_infrared(25),
    queue_front_echo(25),
    queue_rear_echo(25)
    {};

  struct Data {
    uint16_t left_encoder;
    uint16_t right_encoder;
    bool is_charging;
    unsigned char power_percent;
    uint16_t front_echo;
    uint16_t rear_echo;
//    uint16_t right_echo;
    uint16_t front_infrared;
    uint16_t rear_infrared;

    bool stopped;
    float left_motor_current;
    float right_motor_current;

    uint32_t timestamp;
    uint16_t error_state;

    uint8_t version;

  } data;


//  infrared queue
  lqueue <uint> queue_front_infrared;
  lqueue <uint> queue_rear_infrared;

//  echo queue
  lqueue <uint> queue_front_echo;
  lqueue <uint> queue_rear_echo;





  struct Flags {
      // Charging source
      // - first four bits distinguish between adapter or docking base charging
      static const uint8_t AdapterType  = 0x10;
      // - last 4 bits specified the charging status (see Battery.hpp for details)
      static const uint8_t BatteryStateMask = 0x0F;
      static const uint8_t Discharging  = 0x00;
      static const uint8_t Charged      = 0x02;
      static const uint8_t Charging     = 0x06;


    };


  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
  void build_special_variable(float &variable, ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace xbot

#endif /* XBOT_CORE_SENSORS_HPP__ */
