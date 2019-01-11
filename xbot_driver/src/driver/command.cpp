/**
 * @file src/driver/command.cpp
 *
 * @brief Implementation of the command packets.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/command.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Static variables initialization
*****************************************************************************/

const unsigned char Command::header0 = 0xaa;
const unsigned char Command::header1 = 0x55;

/*****************************************************************************
** Implementation [Command Generators]
*****************************************************************************/






Command Command::SetVelocityControl(DiffDrive& diff_drive)
{
  Command outgoing;
  std::vector<float> velocity_commands = diff_drive.velocityCommands();
  outgoing.data.speed = velocity_commands[0];
  outgoing.data.radius = velocity_commands[1];
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetVelocityControl(const float &speed, const float &radius)
{
  Command outgoing;
  outgoing.data.speed = speed;
  outgoing.data.radius = radius;
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetLiftControl(const unsigned char &lift_height)
{
    Command outgoing;
    outgoing.data.lift_height = lift_height;
    outgoing.data.command = Command::Lift;
    return outgoing;


}

Command Command::SetYawPlatformControl(const int &yaw_platform_degree)
{
  Command outgoing;
  outgoing.data.yaw_platform_degree = yaw_platform_degree+120;
  outgoing.data.command = Command::YawPlatform;
  return outgoing;

}

Command Command::SetPitchPlatformControl(const int &pitch_platform_degree)
{
  Command outgoing;
  outgoing.data.pitch_platform_degree = pitch_platform_degree+120;
  outgoing.data.command = Command::PitchPlatform;
  return outgoing;

}

Command Command::SetSoundControl(const bool &sound_state)
{
  Command outgoing;
  outgoing.data.sound_state = sound_state;
  outgoing.data.command = Command::Sound;
  return outgoing;

}

Command Command::SetLedControl(const unsigned char &led)
{
  Command outgoing;
  outgoing.data.led = led;
  outgoing.data.command = Command::StateLed;
  return outgoing;

}


Command Command::SetPowerControl(const bool &power_state)
{
    Command outgoing;
    outgoing.data.power_state = power_state;
    outgoing.data.command = Command::Power;
    return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * Clears the command buffer and resets the header.
 */
void Command::resetBuffer(Buffer& buffer) {
  buffer.clear();
  buffer.resize(64);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
  buffer.push_back(0); // just initialise, we usually write in the payload here later (size of payload only, not stx, not etx, not length)
}

bool Command::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  // need to be sure we don't pass through an emum to the Trans'd buildBytes functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  unsigned char lift_label = 1;
  switch (data.command)
  {
    case Power:
      buildBytes(cmd, byteStream);
      buildBytes(data.power_state, byteStream);
      break;
    case BaseControl:
      buildBytes(cmd, byteStream);
      buildBytes(data.speed, byteStream);
      buildBytes(data.radius, byteStream);
      break;
    case Lift:
      buildBytes(cmd, byteStream);
      buildBytes(lift_label, byteStream);
      buildBytes(data.lift_height, byteStream);
      break;
    case YawPlatform:
      buildBytes(cmd, byteStream);
      buildBytes(data.yaw_platform_degree, byteStream);
      break;
    case PitchPlatform:
      buildBytes(cmd, byteStream);
      buildBytes(data.pitch_platform_degree, byteStream);
      break;
    case Sound:
      buildBytes(cmd, byteStream);
      buildBytes(data.sound_state, byteStream);
      break;
    case StateLed:
      buildBytes(cmd,byteStream);
      buildBytes(data.led, byteStream);
      break;
    default:
      return false;
      break;
  }
  return true;
}



} // namespace xbot
