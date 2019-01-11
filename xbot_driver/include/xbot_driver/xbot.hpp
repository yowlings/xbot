/**
 * @file include/xbot_driver/xbot.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_HPP_
#define XBOT_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <iomanip>
#include <ecl/config.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "parameters.hpp"
#include "command.hpp"
#include "modules.hpp"
#include "packets.hpp"
#include "packet_handler/packet_finder.hpp"
#include "macros.hpp"


/*****************************************************************************
** Extern Templates
*****************************************************************************/

#ifdef ECL_IS_WIN32
  /* Help windows create common instances of sigslots across xbot dll
   * and end user program (otherwise it creates two separate variables!) */
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<const xbot::VersionInfo&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<const std::string&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<xbot::Command::Buffer&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<xbot::PacketFinderBase::BufferType&>;
#endif

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/

union union_sint16
{
  short word;
  unsigned char byte[2];
};

/*****************************************************************************
** Parent Interface
*****************************************************************************/
// 串口数据解码程序，使用有限状态机原理
class PacketFinder : public PacketFinderBase
{
public:
  virtual ~PacketFinder() {}
  //重定义校验函数
  bool checkSum();
};

/*****************************************************************************
 ** Interface [Xbot]
 *****************************************************************************/
/**
 * @brief  The core xbot driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class xbot_PUBLIC Xbot
{
public:
  Xbot();
  ~Xbot();

  /*********************
   ** Configuration
   **********************/
  //初始化
  void init(Parameters &parameters) throw (ecl::StandardException);
  bool base_isAlive() const { return base_is_alive; } /**< Whether the connection to the robot is alive and currently streaming. **/
  bool sensor_isAlive() const { return sensor_is_alive;}
  bool isShutdown() const { return shutdown_requested; } /**< Whether the worker thread is alive or not. **/
  bool isEnabled() const { return is_enabled; } /**< Whether the motor power is enabled or disabled. **/
  bool enable(); /**< Enable power to the motors. **/
  bool disable(); /**< Disable power to the motors. **/
  void shutdown() { shutdown_requested = true; } /**< Gently terminate the worker thread. **/
  void resetXbotState();
  bool is_base_connected() const{ return base_is_connected;}
  bool is_sensor_connected() const{ return sensor_is_connected;}

  /******************************************
  ** Base Packet Processing
  *******************************************/
  //底盘数据流控制程序，运行于base_thread线程中
  void base_spin();
  //底盘修复数据段位
  void base_fixPayload(ecl::PushAndPop<unsigned char> & byteStream);

  /******************************************
  ** Sensor Packet Processing
  *******************************************/
  //机器上方传感器数据流控制程序，运行于sensor_thread线程中
  void sensor_spin();
  //传感器数据段位修复
  void sensor_fixPayload(ecl::PushAndPop<unsigned char> & byteStream);

  /******************************************
  ** Getters - Base Data Protection
  *******************************************/
//  底盘数据锁
  void base_lockDataAccess();
  void base_unlockDataAccess();

  /******************************************
  ** Getters - Data Protection
  *******************************************/
//  传感器数据锁
  void sensor_lockDataAccess();
  void sensor_unlockDataAccess();

  /******************************************
  ** Getters - User Friendly Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
//  获取机器人朝向，若参数use_imu设置为true，则返回IMU的yaw数据
  float getHeading() const;
//  用于调试数据的显示
  int getDebugSensors() const;
//  获取角速度
  float getAngularVelocity() const;
//  获取水平转台角度
  unsigned char getYawPlatformDegree() const;
//  获取竖直转台角度
  unsigned char getPitchPlatformDegree() const;
//  获取急停开关状态量
  bool getStopButtonState() const;
//  获取电机电源状态
  bool getPowerState(){return Power;}

  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
//  底盘核心数据
  CoreSensors::Data getCoreSensorData() const { return core_sensors.data; }
//  传感器数据
  Sensors::Data getExtraSensorsData() const {return sensors.data;}
  /*********************
  ** Feedback
  **********************/
//  获取关节状态数据，用于ros的state_publisher
  void getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate,
                           float &wheel_right_angle, float &wheel_right_angle_rate);
//  机器人码盘数据计算运动轨迹，用于ros发布odom
  void updateOdometry(ecl::Pose2D<double> &pose_update,
                      ecl::linear_algebra::Vector3d &pose_update_rates);

  /*********************
  ** Soft Commands
  **********************/
  void resetOdometry();

  /*********************
  ** Hard Commands
  **********************/
  //速度控制
  void setBaseControl(const float &linear_velocity, const float &angular_velocity);
  //升降控制
  void setLiftControl(const unsigned char &height_percent);
  //水平云台控制
  void setYawPlatformControl(const int &yaw_degree);
  //竖直云台控制
  void setPitchPlatformControl(const int &pitch_degree);
  //电源控制
  void setPowerControl(const bool &power);
  //声音控制
  void setSoundEnableControl(const bool &sound);
  //状态灯控制
  void setLedControl(const char &led);

  //重置机器人
  void resetXbot();

private:
  /*********************
  ** Thread
  **********************/
  //底盘线程
  ecl::Thread base_thread;
// 传感器线程
  ecl::Thread sensor_thread;
  bool shutdown_requested; // helper to shutdown the worker thread.

  /*********************
  ** Record RobotState
  **********************/
  unsigned char HeightPercent;
  bool Power;


  /*********************
  ** Odometry
  **********************/
  DiffDrive diff_drive;
  bool is_enabled;

  /*********************
  ** Inertia
  **********************/
//  机器人初始化的偏置角
  float heading_offset;

  /*********************
  ** Driver Paramters
  **********************/
  Parameters parameters;
//  底盘串口是否连接成功
  bool base_is_connected;
//  传感器串口是否连接
  bool sensor_is_connected;

  /*********************
  ** Acceleration Limiter
  **********************/
  AccelerationLimiter acceleration_limiter;

  /*********************
  ** Packet Handling
  **********************/
  CoreSensors core_sensors;
  Sensors sensors;
//底盘串口与串口buffer
  ecl::Serial base_serial;
  PacketFinder base_packet_finder;
  PacketFinder::BufferType base_data_buffer;

//传感器串口与buffer
  ecl::Serial sensor_serial;
  PacketFinder sensor_packet_finder;
  PacketFinder::BufferType sensor_data_buffer;
  bool base_is_alive; // used as a flag set by the data stream watchdog
  bool sensor_is_alive;


  /*********************
  ** Commands
  **********************/
  void sendBaseControlCommand();
  void sendCommand(Command command);
  ecl::Mutex base_command_mutex; // protection against the user calling the command functions from multiple threads
  ecl::Mutex base_data_mutex;

  ecl::Mutex sensor_command_mutex;
  ecl::Mutex sensor_data_mutex;

  Command xbot_command; // used to maintain some state about the command history
  Command::Buffer command_buffer;

  /*********************
  ** Signals
  **********************/
  ecl::Signal<> base_sig_stream_data;
  ecl::Signal<> sensor_sig_stream_data;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;
  ecl::Signal<Command::Buffer&> base_sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<Command::Buffer&> sensor_sig_raw_data_command;
  ecl::Signal<PacketFinder::BufferType&> base_sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<PacketFinder::BufferType&> sensor_sig_raw_data_stream;
};

} // namespace xbot

#endif /* XBOT_HPP_ */
