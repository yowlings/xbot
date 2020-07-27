#ifndef XBOT_TALKER_ROS_H_
#define XBOT_TALKER_ROS_H_
#include <std_msgs/Bool.h>
#include <iostream>
#include "awaken/awaken_offline.h"
#include "asr/baidu/asr_online.h"
#include "file_operation.h"
#include "nlp/nlp_feedback.h"
#include "tts/text_to_speech.h"
#include "xbot_talker/awaken_status.h"
#include "xbot_talker/keyword_config.h"
#include "xbot_talker/nlpdialog_config.h"
#include "xbot_talker/online_asr_result.h"
#include "xbot_talker/recog_result.h"
#include "xbot_talker/xbot_tts.h"
#include "asr/xunfei/xfei_speech_recog.h"
#include "ros/ros.h"
#include "xbot_talker/play.h"
#include "xbot_talker/chat.h"
// 与asr模块相关的ROS操作的封装类
class ASRModuleRos
{
public:
  ASRModuleRos() : offline_result_vector(2)
  {
  }
  ~ASRModuleRos(){};
  std::string offline_recog_result;
  std::string baidu_online_result;
  std::string xfei_online_result;

  bool init();
  void offlineRecog();
  void onlineRecogXfei();
  void onlineRecogBaidu();

private:
  ros::NodeHandle asr_nodehandle;

  void advertiseTopics();
  void subscribeTopics();
  void advertiseService();

  ros::ServiceServer chat_server_;
  bool requestChat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res);


  ros::ServiceServer keyword_config_service;
  bool requestKeywordConfig(xbot_talker::keyword_config::Request& req, xbot_talker::keyword_config::Response& res);

  /*********************
   ** Ros Publishers
   **********************/
  ros::Publisher recog_result_pub;
  ros::Publisher online_result_pub;
  ros::Publisher sound_mute;
  /*********************
  ** Ros Publisher Function
  **********************/
  void pubOfflineRecogResult(const std::string result, const int accuracy);
  void pubOnlineRecogResult(const std::string result);
  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber is_awaken_sub;

  /*********************
  ** Ros Subscribers Callbacks
  **********************/
  void subscribeAwakenStatus(const xbot_talker::awaken_status);

  /*********************
  ** ASR variable
  **********************/
  bool enable_xfei_online;
  bool enable_baidu_online;
  bool enable_offline;
  bool use_pcm_file;
  bool use_mic;
  bool enable_record_save;
  std::string asr_params_;
  std::string xunfei_online_asr_params_;
  std::string base_path;
  std::string grammar_path;
  std::string config_path;
  std::string pcm_file;
  std::string audio_save_path;
  std::string log_path;
  int audio_channel;
  int log_count_ = 0;
  int online_log_count_ = 0;
  float record_time;
  char* recog_result_json;
  std::vector<std::string> offline_result_vector;
  struct DataBuff pcm_buff = { NULL, 0 };
  FileOperation config_file;
  XfeiSpeechRecog xfei_sr_offline;
  XfeiSpeechRecog xfei_sr_online;
  BaiduAsrOnline baidu_sr_online;
  CSVOperation csv_file;

};

// 与nlp模块相关的ROS操作的封装类
class NLPModuleRos
{
public:
  NLPModuleRos();
  ~NLPModuleRos();
  bool init();
  void pubStartRecog(bool is_awaken, bool enable_chat);

private:
  ros::NodeHandle nlp_nodehandle;

  void advertiseTopics();
  void subscribeTopics();
  void advertiseService();

  /*********************
   ** Ros Publishers
   **********************/
  ros::Publisher start_awaken_pub;  // 开始语音唤醒
  ros::Publisher start_recog_pub;   // 开始离线命令词识别
  ros::Publisher mov_control_pub;   // 机器人移动控制
  ros::Publisher left_put_pub;
  ros::Publisher right_put_pub;

  ros::Publisher left_get_pub;
  ros::Publisher right_get_pub;

  ros::Publisher left_grip_pub;
  ros::Publisher right_grip_pub;

  ros::Publisher welcome_kp_pub;
  ros::Publisher welcome_yes_pub;

  ros::ServiceServer dialog_config_service;
  bool requestDialogConfig(xbot_talker::nlpdialog_config::Request& req, xbot_talker::nlpdialog_config::Response& res);
  /*********************
  ** Ros Publisher Function
  **********************/
  void pubMoveControl(const int robot_action);
  void pubArmControl(const int robot_action);
  void pubWelcomeYes(bool enable_welcome);
  void pubWelcomeKp(const std::string kp_name);

  void pubStartAwaken(bool enable_awake);
  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber recog_result_sub;
  ros::Subscriber awaken_result_sub;
  ros::Subscriber online_result_sub;

  /*********************
  ** Ros Subscribers Callbacks
  **********************/
  void subscribeOfflineRecogResult(const xbot_talker::recog_result);
  void subscribeOnlineRecogResult(const xbot_talker::online_asr_result);

  /*********************
  ** NLP variable
  **********************/
  std::string base_path;
  std::string nlp_config_path;
  std::string config_path;
  std::string log_path;
  int audio_channel;
  bool enable_chat_ = true;
  int fail_count_ = 0;
  int log_count_ = 0;
  std::string tuling_answer_text_;
  ResultFeedback result_handle;
  TuLingRobot tuling_robot;
};

// 与awaken模块相关的ROS操作的封装类
class AwakenModuleRos
{
public:
  AwakenModuleRos();
  ~AwakenModuleRos();
  bool init();
  void isWaken();

private:
  AwakenOffline awaken_module;
  ros::NodeHandle awaken_nodehandle;

  void advertiseTopics();
  void subscribeTopics();
  void subscribeStartAwaken(const std_msgs::BoolConstPtr);
  void pubStartRecog(bool is_awaken);
  bool enable_record_save;
  std::string awaken_params_;
  std::string base_path;
  std::string config_path;
  std::string pcm_file;
  std::string awaken_mode;
  std::string audio_save_path;
  std::string log_path;
  int audio_channel;
  int record_time;

  ros::Publisher is_awaken_pub;
  ros::Subscriber start_awaken_sub;
};

// 与tts模块相关的ROS操作的封装类
class TTSModuleRos
{
public:
  TTSModuleRos();
  ~TTSModuleRos();
  bool init();

private:
  ros::NodeHandle tts_nodehandle;

  void responseServices();
  void requestServices();

  /*********************
   ** Ros TTS RequestServiceClient
   **********************/
  ros::ServiceClient tts_client;

  /*********************
   ** Ros TTS ResponseServiceServer
   **********************/
  ros::ServiceServer tts_server;
  ros::ServiceServer play_server;

  bool playCallback(xbot_talker::play::Request &req, xbot_talker::play::Response &res);


  /*********************
  ** Ros TTS ServiceServer Callbacks
  **********************/
  bool ttsCallback(xbot_talker::xbot_tts::Request& req, xbot_talker::xbot_tts::Response& res);

  /*********************
  ** TTS variable
  **********************/
  std::string base_path;
  std::string audio_save_path;
  std::string log_path;
  int audio_channel;
};
#endif
