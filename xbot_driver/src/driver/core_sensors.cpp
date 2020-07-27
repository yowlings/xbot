/**
 * @file /xbot_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/core_sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"
#include <time.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  buildBytes(Header::CoreSensors, byteStream);
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



bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//    std::cout<<byteStream.size()<<std::endl;

//  if (byteStream.size() < length+2)
//  {
//    std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }
//  标志位，固定0x10
  unsigned char header_id;//0x10 marker
  buildVariable(header_id, byteStream);

  if( header_id != Header::CoreSensors )
  {
      std::cout<<"header_id is wrong. header_id:"<<(unsigned int)header_id<<std::endl;

      return false;
  }
//  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;

  uint16_t tempvariable = 0;
  uint8_t tmp = 0;
//  码盘
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);

//  电源，充电状态与电量

  buildVariable(tempvariable, byteStream);//充电电流，单位10mA

  data.is_charging = (tempvariable < 0)?false:true;
  buildVariable(data.power_percent,byteStream);



//  前后各一路超声，均值滤波，25个邻域数据取均值，每秒50帧数据，取0.5s内的平均

  buildVariable(tempvariable, byteStream); //front_left not used

  buildVariable(tempvariable, byteStream);
  queue_front_echo.lpush(tempvariable);
  data.front_echo = queue_front_echo.mean();

  buildVariable(tempvariable, byteStream); //front_right not used


  buildVariable(tempvariable, byteStream);//rear_left not used

  buildVariable(tempvariable, byteStream);
  queue_rear_echo.lpush(tempvariable);
  data.rear_echo = queue_rear_echo.mean();

  buildVariable(tempvariable, byteStream);//rear_right not used

//  前后各一路红外，均值滤波，25个邻域数据取均值，每秒50帧数据，取0.5s内的平均
  buildVariable(tempvariable, byteStream);//front left not used

  buildVariable(tempvariable, byteStream);
  queue_front_infrared.lpush(tempvariable);
  data.front_infrared = queue_front_infrared.mean();

  buildVariable(tempvariable, byteStream);//front right not used
  buildVariable(tempvariable, byteStream);//rear left not used

  buildVariable(tempvariable, byteStream);
  queue_rear_infrared.lpush(tempvariable);
  data.rear_infrared = queue_rear_infrared.mean();

  buildVariable(tempvariable, byteStream);//rear right not used


//  急停开关状态反馈
  buildVariable(data.motor_enabled,byteStream);

//  两路电机电流
  buildVariable(data.left_motor_current, byteStream);
  buildVariable(data.right_motor_current, byteStream);

//  时间戳，4字节无符号整形，单位us
  buildVariable(data.timestamp, byteStream);

//  故障状态显示
  buildVariable(data.error_state, byteStream);

  //软件版本
  buildVariable(data.version, byteStream);


  return true;
}

void CoreSensors::build_special_variable(float &variable, ecl::PushAndPop<unsigned char> &byteStream)
{
    if (byteStream.size() < 2)
      return;

    unsigned char a, b;
    buildVariable(a,byteStream);
    buildVariable(b,byteStream);
    variable = ((unsigned int)(a&0x0f))/100.0;

    variable += ((unsigned int)(a>>4))/10.0;

    variable += (unsigned int)(b&0x0f);

    variable += ((unsigned int)(b>>4))*10;


}


} // namespace xbot
