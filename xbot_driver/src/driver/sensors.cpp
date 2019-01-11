/**
 * @file /xbot_driver/src/driver/imu_sensors.cpp
 *
 * @brief Implementation of the imu sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_imu/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool Sensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  buildBytes(Header::ImuSensors, byteStream);
//  buildBytes(length, byteStream);
//  buildBytes(data.time_stamp, byteStream);	//2
//  buildBytes(data.bumper, byteStream);		//1
//  buildBytes(data.wheel_drop, byteStream);	//1
//  buildBytes(data.cliff, byteStream);		//1
//  buildBytes(data.left_encoder, byteStream);	//2
//  buildBytes(data.right_encoder, byteStream);	//2
//  buildBytes(data.left_pwm, byteStream);	//1
//  buildBytes(data.right_pwm, byteStream);	//1
//  buildBytes(data.buttons, byteStream);		//1
//  buildBytes(data.charger, byteStream);		//1
//  buildBytes(data.battery, byteStream);		//1
//  buildBytes(data.over_current, byteStream);	//1

  return true;
}
bool Sensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  if (byteStream.size() < length+2)
//  {
//      std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }

  unsigned char header_id;
  buildVariable(header_id, byteStream);
//标志位,0x11
  if( header_id != Header::Sensors )
  {
      std::cout<<"header_id is wrong. header_id:"<<(unsigned int)header_id<<std::endl;

      return false;
  }
//  水平转台角度
  buildVariable(data.yaw_platform_degree, byteStream);
//  竖直转台角度
  buildVariable(data.pitch_platform_degree, byteStream);
//  音量控制
  buildVariable(data.sound_status, byteStream);
//  IMU9250九轴裸数据
  buildVariable(data.acc_x, byteStream);
  buildVariable(data.acc_y, byteStream);
  buildVariable(data.acc_z, byteStream);
  buildVariable(data.gyro_x, byteStream);
  buildVariable(data.gyro_y, byteStream);
  buildVariable(data.gyro_z, byteStream);
  buildVariable(data.mag_x, byteStream);
  buildVariable(data.mag_y, byteStream);
  buildVariable(data.mag_z, byteStream);

//  IMU9250计算出的三轴角度
  buildVariable(data.yaw,byteStream);
  buildVariable(data.pitch, byteStream);
  buildVariable(data.roll, byteStream);
//  IMU9250计算出的四元数
  buildVariable(data.q1, byteStream);
  buildVariable(data.q2, byteStream);
  buildVariable(data.q3, byteStream);
  buildVariable(data.q4, byteStream);

//  舵机故障状态
  buildVariable(data.error_status, byteStream);
//  时间戳，0~65536，单位us
  buildVariable(data.timestamp, byteStream);

  return true;
}



} // namespace xbot
