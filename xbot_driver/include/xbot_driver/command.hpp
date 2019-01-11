/**
 * @file include/xbot_driver/command.hpp
 *
 * @brief Command structure.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_COMMAND_DATA_HPP__
#define XBOT_COMMAND_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include "modules.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/


namespace xbot
{

class xbot_PUBLIC Command : public packet_handler::payloadBase
{
public:
  typedef ecl::PushAndPop<unsigned char> Buffer;
  typedef ecl::Stencil< Buffer > BufferStencil;

  /**
   * These values are used to detect the type of sub-payload that is ensuing.
   */
  enum Name
  {
    Power = 1,
    BaseControl = 2,
    YawPlatform=3,
    PitchPlatform = 4,
    Sound = 5,
    StateLed = 6,
    Lift = 7
  };


  /**
   * @brief Data structure containing data for commands.
   *
   * It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data
  {
    Data()
      : command(BaseControl),
        speed(0.0),
        radius(0.0),
        lift_height(0),
        yaw_platform_degree(0),
        pitch_platform_degree(0),
        power_state(0),
        sound_state(0),sound_volume(100),
        led(0)
    {
    }

    Name command;

    // BaseControl
    float speed;
    float radius;
    unsigned char lift_height;
    unsigned char yaw_platform_degree;
    unsigned char pitch_platform_degree;
    unsigned char led;
    unsigned char sound_volume;
    bool sound_state;
    bool power_state;



  };

  virtual ~Command() {}

  static Command SetVelocityControl(DiffDrive& diff_drive);
  static Command SetVelocityControl(const float &speed, const float &radius);

  static Command SetLiftControl(const unsigned char &lift_height);

  static Command SetYawPlatformControl(const int &yaw_platform_degree);

  static Command SetPitchPlatformControl(const int &pitch_platform_degree);

  static Command SetSoundControl(const bool &sound_state);

  static Command SetLedControl(const unsigned char &led);

  static Command SetPowerControl(const bool &power_state);
  Data data;

  void resetBuffer(Buffer &buffer);
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream) { return true; } /**< Unused **/

private:
  static const unsigned char header0;
  static const unsigned char header1;

};

} // namespace xbot

#endif /* XBOT_COMMAND_DATA_HPP__ */

