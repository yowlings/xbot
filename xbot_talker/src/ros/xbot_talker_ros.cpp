#include "ros/xbot_talker_ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <thread>
#include "common_config.h"
#include "asr/xunfei/xfei_speech_recog.h"
/**************************************************
*************   Ros ASR Module  *******************
***************************************************/

bool ASRModuleRos::init()
{
  advertiseTopics();
  subscribeTopics();
  advertiseService();
  // 获取参数
  asr_nodehandle.param("/asr_sample/enable_xfei_online", enable_xfei_online, bool(false));
  asr_nodehandle.param("/asr_sample/enable_baidu_online", enable_baidu_online, bool(false));
  asr_nodehandle.param("/asr_sample/enable_offline", enable_offline, bool(true));

  asr_nodehandle.param("/asr_sample/use_pcm_file", use_pcm_file, bool(false));
  asr_nodehandle.param("/asr_sample/use_mic", use_mic, bool(true));
  asr_nodehandle.param("/asr_sample/enable_record_save", enable_record_save, bool(true));

  asr_nodehandle.param("/asr_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  asr_nodehandle.param("/asr_sample/grammar_path", grammar_path,
                       std::string("~/catkin_ws/src/xbot_talker/cache/grammar_config"));
  asr_nodehandle.param("/asr_sample/pcm_file", pcm_file,
                       std::string("~/catkin_ws/src/xbot_talker/defaultconfig/audio/nihao_test.pcm"));
  asr_nodehandle.param("/asr_sample/audio_save_path", audio_save_path,
                       std::string("~/catkin_ws/src/xbot_talker/cache/audio"));
  asr_nodehandle.param("/asr_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));

  asr_nodehandle.param("/asr_sample/audio_channel", audio_channel, int(1));
  asr_nodehandle.param("/asr_sample/record_time", record_time, float(5));
  // 科大讯飞asr登录以及语法参数配置
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);
  if (enable_offline == true)
  {
    asr_params_ = xunfei_config.configGramParas(base_path, grammar_path);
    xfei_sr_offline.setAsrParams(base_path, pcm_file, asr_params_, audio_channel);
  }
  // 设置科大讯飞在线语音识别相关参数
  if (enable_xfei_online == true)
  {
    xunfei_online_asr_params_ = "sub = iat, domain = iat, language = zh_cn, "
                                "accent = mandarin, sample_rate = 16000, "
                                "result_type = plain, result_encoding = utf8";
    xfei_sr_online.setAsrParams(base_path, pcm_file, xunfei_online_asr_params_, audio_channel);
  }
  if (enable_baidu_online == true)
  {
    // 设置百度在线语音识别相关参数
    baidu_sr_online.setAsrParams(base_path, pcm_file, audio_channel);
    // 百度语音识别模块的初始化,设置asr相关参数
    baidu_sr_online.initAndConfigAsr();
    // 获取百度在线语音识别的Token
    baidu_sr_online.speechGetToken();
  }

  ROS_INFO_STREAM("--------Speech recognition module is ready to be waken up!--------");
}

// 发布的话题
void ASRModuleRos::advertiseTopics()
{
  recog_result_pub = asr_nodehandle.advertise<xbot_talker::recog_result>("xbot_talker/recog_result", 100);
  online_result_pub = asr_nodehandle.advertise<xbot_talker::online_asr_result>("xbot_talker/online_recog_result", 100);
  //sound_mute = asr_nodehandle.advertise<std_msgs::Bool>("mobile_base/commands/sound_enable", 100);
}

// 订阅的话题
void ASRModuleRos::subscribeTopics()
{
  is_awaken_sub = asr_nodehandle.subscribe(std::string("xbot_talker/awaken_status"), 10,
                                           &ASRModuleRos::subscribeAwakenStatus, this);
}

void ASRModuleRos::advertiseService()
{
  keyword_config_service = asr_nodehandle.advertiseService(std::string("/xbot_talker/asr_keyword_config"),
                                                           &ASRModuleRos::requestKeywordConfig, this);
  chat_server_ = asr_nodehandle.advertiseService(std::string("/xbot/chat"),
                                                           &ASRModuleRos::requestChat, this);
}

// 添加新的关键词构建语法后，重新配置语法参数
bool ASRModuleRos::requestKeywordConfig(xbot_talker::keyword_config::Request& req,
                                        xbot_talker::keyword_config::Response& res)
{
  if (req.keyword != "")
  {
    std::string bnf_word;
    bnf_word = config_file.readFileAsString(base_path + "/userconfig/grammar.bnf");
    ROS_INFO_STREAM("BNF file :" << bnf_word);
    CommonConfig& xunfei_config = CommonConfig::get_instance();
    asr_params_ = xunfei_config.configGramParas(base_path, grammar_path);
    res.error_code = 0;
  }
  else
  {
    res.error_code = 1;
  }
}

// 识别结果的发布.
void ASRModuleRos::pubOfflineRecogResult(const std::string result, const int accuracy)
{
  xbot_talker::recog_result recog_result;
  recog_result.recog_result = result;
  recog_result.recog_accuracy = accuracy;
  recog_result_pub.publish(recog_result);
}

// 完整的百度在线语音识别结果的发布.
void ASRModuleRos::pubOnlineRecogResult(const std::string result)
{
  xbot_talker::online_asr_result recog_result;
  recog_result.online_asr_result = result;
  online_result_pub.publish(recog_result);
}

// 科大讯飞离线语音识别线程
void ASRModuleRos::offlineRecog()
{
  recog_result_json = NULL;
  if (enable_offline == true)
  {
    if (use_pcm_file == true)
    {
      xfei_sr_offline.getPcmFileData();
    }
    else if (use_mic == true)
    {
      // 固定时长录音
      pcm_buff = xfei_sr_offline.recordThroughMIC(record_time, enable_record_save);
      xfei_sr_offline.getPCMData(pcm_buff);
    }
    // 科大讯飞识别模块的初始化
    xfei_sr_offline.initAsr();
    // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
    recog_result_json = xfei_sr_offline.dataLoopRecog();

    if (recog_result_json == NULL)
    {
      offline_result_vector[0] = "none_result";
      offline_result_vector[1] = "0";
      csv_file.writeRowData(log_path + "/asr_recog_result_log.csv", offline_result_vector);
    }
    else
    {
      // 在识别结果不为空时，从json结果中解析出需要的结果和置信度
      offline_result_vector = xfei_sr_offline.resultFromJson();
      csv_file.writeRowData(log_path + "/asr_recog_result_log.csv", offline_result_vector);
    }
    // 一次识别结束后释放资源
    xfei_sr_offline.uninitAsr();
  }
}

// 科大讯飞在线语音识别线程
void ASRModuleRos::onlineRecogXfei()
{
  char* recog_result = NULL;
  if (enable_xfei_online == true)
  {
    if (use_pcm_file == true)
    {
      // 获取pcm_file的数据
      xfei_sr_online.getPcmFileData();
    }
    else if (use_mic == true)
    {
      ROS_INFO_STREAM("PCM" << pcm_buff.size << std::endl);
      xfei_sr_online.getPCMData(pcm_buff);
    }
    // 科大讯飞识别模块的初始化
    xfei_sr_online.initAsr();
    // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
    recog_result = xfei_sr_online.dataLoopRecog();
    // 有时会出现离线识别结果置信度很低，但实际上无语音输入的情况，此情况下在线识别结果为空，需要加一个判断
    if (recog_result == NULL)
    {
      xfei_online_result = "none_result";
    }
    else
    {
      xfei_online_result = recog_result;
    }

    ROS_INFO_STREAM("Xunfei online iat result is :" << xfei_online_result << std::endl);

    std::ofstream result_log(log_path + "/asr_online_recog_result_log.txt", std::ios::app);
    if (result_log.is_open())
    {
      result_log << online_log_count_ << ":" << xfei_online_result << std::endl;
      online_log_count_++;
    }
    else
    {
      ROS_ERROR_STREAM("LOG ERROR");
    }
    result_log.close();
    xfei_sr_online.uninitAsr();
  }
}
// 百度在线语音识别线程
void ASRModuleRos::onlineRecogBaidu()
{
  if (enable_baidu_online == true)
  {
    if (use_pcm_file == true)
    {  // 读取pcm文件里的音频数据
      baidu_sr_online.getPcmFileData();
    }
    else if (use_mic == true)
    {
      struct DataBuff pcm_buff = { NULL, 0 };
      // 录音
      pcm_buff = baidu_sr_online.recordThroughMIC(record_time, enable_record_save);
      baidu_sr_online.getPCMData(pcm_buff);
    }
    char* recog_result_online;
    // 百度在线语音识别并获取识别结果
    recog_result_online = baidu_sr_online.runAsrAndRecog();
    // 从完整的json语音识别结果中解析出需要的字符串结果
    baidu_online_result = baidu_sr_online.resultFromJson();
    if (baidu_online_result.empty())
    {
      baidu_online_result = "none_result";
    }
    // 一次识别结束后释放资源
    baidu_sr_online.uninitAsr();
    std::ofstream result_log(log_path + "/asr_online_recog_result_log.txt", std::ios::app);
    if (result_log.is_open())
    {
      result_log << online_log_count_ << ":" << recog_result_online << std::endl;
      online_log_count_++;
    }
    else
    {
      ROS_ERROR_STREAM("LOG ERROR");
    }
    result_log.close();
    ROS_INFO_STREAM("Online asr result is :" << recog_result_online << std::endl);
    ROS_INFO_STREAM("Online asr finall result is :" << baidu_online_result << std::endl);
  }
}

bool ASRModuleRos::requestChat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res){
  if(req.start_chat){
    //播放“嘟”声提示音.
    std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/call.wav";
    system(dialogue_prompt_wav.c_str());
    //std_msgs::Bool enable_sound;
    //enable_sound.data = false;
    //sound_mute.publish(enable_sound);
    offline_recog_result = "";
    baidu_online_result = "";
    xfei_online_result = "";
    ROS_INFO_STREAM("--------Speech recognition is started!--------");
    //std::thread onlineThreadBaidu(&ASRModuleRos::onlineRecogBaidu, this);
    std::thread offlineThread(&ASRModuleRos::offlineRecog, this);
    offlineThread.join();
    //onlineThreadBaidu.join();
    if (std::stoi(offline_result_vector[1]) >= 35)
    {
      // 科大讯飞离线识别结果准确度>=35时，默认为识别准确，直接发布离线识别结果，不再进行在线处理。
      pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
    }
    else if (enable_xfei_online == true && offline_result_vector[0] != "none_result")
    {
      // 如果离线准确度小于35，且离线结果不为空（离线结果为空意味着录音时间段内没有任何语音输入，十分安静），
      // 并且讯飞在线识别功能开启的情况下，调用在线语音进行识别，发布在线识别结果。
      onlineRecogXfei();
      pubOnlineRecogResult(xfei_online_result);
    }
    else if (enable_baidu_online == true && offline_result_vector[0] != "none_result")
    {
      // 同上。同等网络状况下，百度在线识别比科大讯飞在线识别慢很多，甚至因网络状况出现卡顿。
      // 所以建议使用科大讯飞在线和离线结合的方式。
      pubOnlineRecogResult(baidu_online_result);
    }
    else
    {
      pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
    }
    ROS_INFO_STREAM("Baidu online speech recog result: " << baidu_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei online speech recog result: " << xfei_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei offline speech recog result: " << offline_result_vector[0] << std::endl);
    pcm_buff.data = NULL;
    pcm_buff.size = 0;
    //enable_sound.data = true;
    //sound_mute.publish(enable_sound);
  }
  res.chat_success =true;
  return true;
}



// topic:"xbot_talker/awaken_status"的回调函数.
// 当is_awaken为true时，开启语法识别相关功能.
void ASRModuleRos::subscribeAwakenStatus(const xbot_talker::awaken_status msg)
{
  if (msg.is_awaken)
  {
    //播放“嘟”声提示音.
    std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/call.wav";
    system(dialogue_prompt_wav.c_str());
    std_msgs::Bool enable_sound;
    //enable_sound.data = false;
    //sound_mute.publish(enable_sound);
    offline_recog_result = "";
    baidu_online_result = "";
    xfei_online_result = "";
    ROS_INFO_STREAM("--------Speech recognition is awakened!--------");
    std::thread onlineThreadBaidu(&ASRModuleRos::onlineRecogBaidu, this);
    std::thread offlineThread(&ASRModuleRos::offlineRecog, this);
    offlineThread.join();
    onlineThreadBaidu.join();
    if (std::stoi(offline_result_vector[1]) >= 35)
    {
      // 科大讯飞离线识别结果准确度>=35时，默认为识别准确，直接发布离线识别结果，不再进行在线处理。
      pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
    }
    else if (enable_xfei_online == true && offline_result_vector[0] != "none_result")
    {
      // 如果离线准确度小于35，且离线结果不为空（离线结果为空意味着录音时间段内没有任何语音输入，十分安静），
      // 并且讯飞在线识别功能开启的情况下，调用在线语音进行识别，发布在线识别结果。
      onlineRecogXfei();
      pubOnlineRecogResult(xfei_online_result);
    }
    else if (enable_baidu_online == true && offline_result_vector[0] != "none_result")
    {
      // 同上。同等网络状况下，百度在线识别比科大讯飞在线识别慢很多，甚至因网络状况出现卡顿。
      // 所以建议使用科大讯飞在线和离线结合的方式。
      pubOnlineRecogResult(baidu_online_result);
    }
    else
    {
      pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
    }
    ROS_INFO_STREAM("Baidu online speech recog result: " << baidu_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei online speech recog result: " << xfei_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei offline speech recog result: " << offline_result_vector[0] << std::endl);
    pcm_buff.data = NULL;
    pcm_buff.size = 0;
    //enable_sound.data = true;
    //sound_mute.publish(enable_sound);
  }
}

/**************************************************
*************   Ros NLP Module  *******************
***************************************************/
NLPModuleRos::NLPModuleRos()
{
}
NLPModuleRos::~NLPModuleRos()
{
}
bool NLPModuleRos::init()
{
  advertiseTopics();
  subscribeTopics();
  advertiseService();
  // 获取参数
  nlp_nodehandle.param("/nlp_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  nlp_nodehandle.param("/nlp_sample/nlp_config_path", nlp_config_path,
                       std::string("~/catkin_ws/src/xbot_talker/defaultconfig/answer_dic.csv"));

  nlp_nodehandle.param("/nlp_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));

  // 科大讯飞登录
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);
  for (int i = 0; i < 2; i++)
  {
    pubStartAwaken(true);
    sleep(1);
  }
  ROS_INFO_STREAM("--------Result feedback module is ready to start!--------");

  // 读取对话配置文件内容保存进answer_table变量
  result_handle.readAnswerTable(nlp_config_path);
}

void NLPModuleRos::advertiseTopics()
{
  start_recog_pub = nlp_nodehandle.advertise<xbot_talker::awaken_status>("xbot_talker/awaken_status", 100);
  start_awaken_pub = nlp_nodehandle.advertise<std_msgs::Bool>("xbot_talker/enable_awake", 100);
  mov_control_pub = nlp_nodehandle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
  left_get_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/left_get", 10);
  right_get_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/right_get", 10);

  left_put_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/left_put", 10);
  right_put_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/right_put", 10);

  left_grip_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/arm/commands/left_grip", 10);
  right_grip_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/arm/commands/right_grip", 10);
  welcome_yes_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/welcome/yes", 10);
  welcome_kp_pub = nlp_nodehandle.advertise<std_msgs::String>("/welcome/kp", 10);
}

void NLPModuleRos::subscribeTopics()
{
  recog_result_sub = nlp_nodehandle.subscribe(std::string("xbot_talker/recog_result"), 10,
                                              &NLPModuleRos::subscribeOfflineRecogResult, this);
  online_result_sub = nlp_nodehandle.subscribe(std::string("/xbot_talker/online_recog_result"), 10,
                                               &NLPModuleRos::subscribeOnlineRecogResult, this);
}

void NLPModuleRos::advertiseService()
{
  dialog_config_service = nlp_nodehandle.advertiseService(std::string("/xbot_talker/nlp_dialog_config"),
                                                          &NLPModuleRos::requestDialogConfig, this);
}

bool NLPModuleRos::requestDialogConfig(xbot_talker::nlpdialog_config::Request& req,
                                       xbot_talker::nlpdialog_config::Response& res)
{
  if (req.keyword != "back_config")
  {
    bool is_repeate = result_handle.isKeywordRepeated(req.keyword);
    if (is_repeate == false)
    {
      CSVOperation csv_file;
      std::vector<std::string> dialog_add;
      dialog_add.push_back(req.keyword);
      dialog_add.push_back(req.answer);
      dialog_add.push_back(std::to_string(req.action));
      dialog_add.push_back(std::to_string(req.action_mode));
      csv_file.writeRowData(nlp_config_path, dialog_add);
      // 重新读取对话配置文件内容保存进answer_table变量
      result_handle.readAnswerTable(nlp_config_path);
      res.error_code = 0;
      return true;
    }
    else
    {
      res.error_code = 1;
      return false;
    }
  }
  else
  {
    // 重新读取对话配置文件内容保存进answer_table变量
    result_handle.readAnswerTable(nlp_config_path);
    return true;
  }
}

void NLPModuleRos::pubStartRecog(bool is_awaken, bool enable_chat)
{
  xbot_talker::awaken_status awaken_status;
  awaken_status.is_awaken = is_awaken;
  start_recog_pub.publish(awaken_status);
}

void NLPModuleRos::pubStartAwaken(bool enable_awake)
{
  std_msgs::Bool enable_awaken;
  enable_awaken.data = enable_awake;
  start_awaken_pub.publish(enable_awaken);
}

// 控制机械臂动作
void NLPModuleRos::pubArmControl(const int robot_action)
{
  std_msgs::Empty empty_msg;
  std_msgs::Bool bool_msg;
  switch (robot_action)
  {
    case RAISE_LEFT_HAND:
      left_get_pub.publish(empty_msg);
      break;
    case PUT_DOWM_LEFT_HAND:
      left_put_pub.publish(empty_msg);
      break;
    case RAISE_RIGHT_HAND:
      right_get_pub.publish(empty_msg);
      break;
    case PUT_DOWN_RIGHT_HAND:
      right_put_pub.publish(empty_msg);
      break;
    case LEFT_HAND_GRIP:
      bool_msg.data = true;
      left_grip_pub.publish(bool_msg);
      break;
    case LEFT_HAND_OPEN:
      bool_msg.data = false;
      left_grip_pub.publish(bool_msg);
      break;
    case RIGHT_HAND_GRIP:
      bool_msg.data = true;
      right_grip_pub.publish(bool_msg);
      break;
    case RIGHT_HAND_OPEN:
      bool_msg.data = false;
      right_grip_pub.publish(bool_msg);
      break;
  }
}

void NLPModuleRos::pubWelcomeYes(bool enable_welcome)
{
  std_msgs::Bool msg;
  msg.data = enable_welcome;
  welcome_yes_pub.publish(msg);
}

void NLPModuleRos::pubWelcomeKp(const std::string kp_name)
{
  std_msgs::String msg;
  msg.data = kp_name;
  welcome_kp_pub.publish(msg);
}

// 控制机器人进行前进、后退、左转、右转等动作
void NLPModuleRos::pubMoveControl(const int robot_action)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  switch (robot_action)
  {
    case TAKE_A_STEP_FORWARD:
      twist.linear.x = 0.15;
      mov_control_pub.publish(twist);
      break;

    case TAKE_A_STEP_BACKWARD:
      twist.linear.x = -0.15;
      mov_control_pub.publish(twist);

      break;
    case TURN_LEFT:
      twist.angular.z = 0.78;
      mov_control_pub.publish(twist);
      break;

    case TURN_RIGHT:
      twist.angular.z = -0.78;
      mov_control_pub.publish(twist);
      break;
  }
}

void NLPModuleRos::subscribeOfflineRecogResult(const xbot_talker::recog_result msg)
{
  ROS_INFO_STREAM("recog_result is :" << msg.recog_result << " |recog_accuracy is :" << msg.recog_accuracy
                                      << std::endl);

  std::ofstream result_log(log_path + "/recog_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << log_count_ << ":" << msg.recog_result << std::endl;
    log_count_++;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();
  // 判断识别结果是否有效.
  bool ret = result_handle.resultIsValid(msg.recog_result);
  if (ret == false)
  {
    // 无效的识别结果意味着录音时间段内没有人说话，播放“没有人说话吗？那我先退下了”提示音并进入唤醒模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/none_speech.wav";
    system(sorry_prompt_wav.c_str());
    //enable_chat_ = false;
    pubStartAwaken(true);
    //pubStartRecog(true, true);
  }
  else if (msg.recog_accuracy < 15)
  {
    // 离线识别准确度小于15,可认定有人说话但说的与定义好的语法关键词没有任何关系
    // 播放“不知道怎么回答了，请重新唤醒我吧”提示音，并进入唤醒模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/dontknow.wav";
    system(sorry_prompt_wav.c_str());
    //enable_chat_ = false;
    pubStartAwaken(true);
    pubStartRecog(true, true);
  }
  else if (msg.recog_accuracy < 35)
  {
    // 离线识别准确度在20-35之间，可认为用户说了语法关键词，但因为噪声等原因导致准确度很低，为避免出现误操作，需要提示用户确认一遍
    // 播放“识别不够准确，请再说一遍命令词吧”提示音，并进入语音识别模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/low_SC.wav";
    system(sorry_prompt_wav.c_str());
    if (enable_chat_)
    {
      pubStartRecog(true, true);
    }
    else
    {
      pubStartRecog(true, false);
    }
  }
  else
  {
    // 根据定义的应答策略检测识别到的关键词的识别策略.
    result_handle.keywordDetection();
    // 根据应答策略进行响应.
    result_handle.keywordResponse(base_path + "/cache/wav");

    // 监测是否需要机器人控制以及控制类型
    int robot_action = result_handle.actionDetect();
    if (robot_action == START_CHAT)
    {
      enable_chat_ = true;
      pubStartRecog(true, true);
    }
    else if ((robot_action >= 3) && (robot_action <= 14))
    {
      if (robot_action < 7)
      {
        pubMoveControl(robot_action);
        sleep(1);
      }
      else if (robot_action >= 7)
      {
        pubArmControl(robot_action);
        sleep(1);
      }
      if (enable_chat_)
      {
        pubStartRecog(true, true);
      }
      else
      {
        //pubStartAwaken(true);
        pubStartRecog(true, true);

      }
    }
    else if (robot_action == CLOSE_CHAT)
    {
      pubStartRecog(false, false);
      ROS_INFO_STREAM("-------- The conversation is closed! --------");
      ROS_INFO_STREAM("--If you want to start asr module, say the awaken_word again! --");
      //pubStartAwaken(true);
      enable_chat_ = true;
    }
    else if (robot_action == BACK_TO_ORIGINAL_LOCATION)
    {
      pubWelcomeYes(true);
      pubWelcomeKp("kp_origin");
      pubStartRecog(false, false);
      pubStartAwaken(true);
      enable_chat_ = true;
    }
    else if (robot_action == NAVI_TO_FANGZONG)
    {
      pubWelcomeYes(true);
      pubWelcomeKp("kp_fangyang");
      pubStartRecog(true, false);
      //pubStartAwaken(true);
      enable_chat_ = true;
    }
    else if (robot_action == NAVI_TO_CHANGZONG)
    {
      pubWelcomeYes(true);
      pubWelcomeKp("kp_cxm");
      pubStartRecog(true, false);
      //pubStartAwaken(true);
      enable_chat_ = true;
    }
    else
    {
      // 控制asr开启新一轮语音识别
      if (enable_chat_)
      {
        pubStartRecog(true, true);
      }
      else
      {
        pubStartAwaken(true);
      }
    }
  }
  result_handle.uninitNLP();
}

void NLPModuleRos::subscribeOnlineRecogResult(const xbot_talker::online_asr_result msg)
{
  tuling_answer_text_ = "";
  ROS_INFO_STREAM("online asr result is :" << msg.online_asr_result << std::endl);
  std::ofstream result_log(log_path + "/recog_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << log_count_ << ":" << msg.online_asr_result << std::endl;
    log_count_++;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();

  // 判断识别结果是否有效.
  bool ret = result_handle.resultIsValid(msg.online_asr_result);
  if (ret == false)
  {
    // 无效的识别结果意味着录音时间段内没有人说话，播放“没有人说话吗？那我先退下了”提示音并进入唤醒模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/none_speech.wav";
    system(sorry_prompt_wav.c_str());
    pubStartAwaken(true);
    pubStartRecog(true, true);
    enable_chat_ = true;
  }
  else
  {
    // 设置图灵机器人对话请求参数
    tuling_robot.setAskJson(msg.online_asr_result);
    // 请求图灵机器人接口，获取回答
    tuling_robot.callTulingApi();
    // 从图灵机器人的完整json回答中解析出最终需要的text回答
    tuling_answer_text_ = tuling_robot.textFromJson();
    ROS_INFO_STREAM("TuLing robot answer is:" << tuling_answer_text_ << std::endl);
    // 一次回答后释放资源
    tuling_robot.uninitTuling();
    result_handle.resultIsValid(tuling_answer_text_);
    bool has_answer = result_handle.keywordDetection();

    if (has_answer == true)
    {
      // 根据应答策略进行响应.
      result_handle.keywordResponse(base_path + "/cache/wav");

      // 监测是否需要机器人控制以及控制类型
      int robot_action = result_handle.actionDetect();
      if (robot_action == START_CHAT)
      {
        enable_chat_ = true;
        pubStartRecog(true, true);
      }
      else if ((robot_action >= 3) && (robot_action <= 14))
      {
        if (robot_action < 7)
        {
          pubMoveControl(robot_action);
          sleep(1);
        }
        else if (robot_action >= 7)
        {
          pubArmControl(robot_action);
          sleep(1);
        }
        if (enable_chat_)
        {
          pubStartRecog(true, true);
        }
        else
        {
          pubStartAwaken(true);
        }
      }
      else if (robot_action == CLOSE_CHAT)
      {
        pubStartRecog(false, false);
        ROS_INFO_STREAM("-------- The conversation is closed! --------");
        ROS_INFO_STREAM("--If you want to start asr module, say the awaken_word again! --");
        pubStartAwaken(true);
        //enable_chat_ = false;
      }
      else if (robot_action == BACK_TO_ORIGINAL_LOCATION)
      {
        pubWelcomeYes(true);
        pubWelcomeKp("kp_origin");
        pubStartRecog(false, false);
        pubStartAwaken(true);
        //enable_chat_ = false;
      }
      else if (robot_action == NAVI_TO_FANGZONG)
      {
        pubWelcomeYes(true);
        pubWelcomeKp("kp_fangyang");
        pubStartRecog(false, false);
        pubStartAwaken(true);
        //enable_chat_ = false;
      }
      else if (robot_action == NAVI_TO_CHANGZONG)
      {
        pubWelcomeYes(true);
        pubWelcomeKp("kp_cxm");
        pubStartRecog(true, false);
        pubStartAwaken(true);
        //enable_chat_ = false;
      }
      else
      {
        // 控制asr开启新一轮语音识别
        if (enable_chat_)
        {
          pubStartRecog(true, true);
        }
        else
        {
          pubStartAwaken(true);
          pubStartRecog(true, true);
        }
      }
    }
    else
    {
      TextToSpeech text_to_speech;
      bool ret;
      ret = text_to_speech.audioConverter(base_path + "/cache/wav", tuling_answer_text_.c_str());
      if (enable_chat_)
      {
        pubStartRecog(true, true);
      }
      else
      {
        pubStartAwaken(true);
      }
    }
  }
  result_handle.uninitNLP();
}

/**************************************************
*************   Ros Awaken Module  *******************
***************************************************/
AwakenModuleRos::AwakenModuleRos()
{
}
AwakenModuleRos::~AwakenModuleRos()
{
}

bool AwakenModuleRos::init()
{
  advertiseTopics();
  subscribeTopics();
  // 获取参数
  awaken_nodehandle.param("/awaken_sample/awaken_mode", awaken_mode, std::string("mic"));
  awaken_nodehandle.param("/awaken_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  awaken_nodehandle.param("/awaken_sample/pcm_file", pcm_file,
                          std::string("~/catkin_ws/src/xbot_talker/defaultconfig/audio/awaken.pcm"));
  awaken_nodehandle.param("/awaken_sample/enable_record_save", enable_record_save, bool(true));

  awaken_nodehandle.param("/awaken_sample/audio_save_path", audio_save_path,
                          std::string("~/catkin_ws/src/xbot_talker/cache/"));
  awaken_nodehandle.param("/awaken_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/"));

  awaken_nodehandle.param("/awaken_sample/audio_channel", audio_channel, int(1));
  awaken_nodehandle.param("/awaken_sample/record_time", record_time, int(60));

  awaken_module.loginAndSetParams(base_path, pcm_file, audio_channel);

  ROS_INFO_STREAM("--------Voice wake up module is ready to be waken up!--------");
}

void AwakenModuleRos::advertiseTopics()
{
  is_awaken_pub = awaken_nodehandle.advertise<xbot_talker::awaken_status>("xbot_talker/awaken_status", 100);
}

void AwakenModuleRos::subscribeTopics()
{
  start_awaken_sub = awaken_nodehandle.subscribe(std::string("xbot_talker/enable_awake"), 10,
                                                 &AwakenModuleRos::subscribeStartAwaken, this);
}

void AwakenModuleRos::pubStartRecog(bool is_awaken)
{
  xbot_talker::awaken_status awaken_status;
  awaken_status.is_awaken = is_awaken;
  is_awaken_pub.publish(awaken_status);
}

void AwakenModuleRos::isWaken()
{
  while (true)
  {
    if (awaken_module.is_awaken)
    {
      if (awaken_mode == "mic")
      {
        awaken_module.stopRecordThroughMIC();
      }
      if (enable_record_save == true)
      {
        awaken_module.saveRecordDataToFile();
      }
      // 播放提示音提醒唤醒成功
      std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/wozai.wav";
      system(dialogue_prompt_wav.c_str());
      // 开启离线命令词识别
      pubStartRecog(true);
      awaken_module.is_awaken = false;
      break;
    }
    else
    {
      continue;
    }
  }
}

void AwakenModuleRos::subscribeStartAwaken(const std_msgs::BoolConstPtr msg)
{
  if (msg->data == true)
  {
    std::thread check(&AwakenModuleRos::isWaken, this);
    awaken_module.awakenInit();
    if (awaken_mode == "file")
    {
      awaken_module.getPcmFileData();
      awaken_module.dataLoopAwaken();
    }
    else
    {
      awaken_module.recordThroughMIC(record_time, enable_record_save);
      // TODO:可打断模式下，需要改为在此处关闭录音,取消下行注释即可
      // awaken_module.stopRecordThroughMIC();
    }
    check.join();
    awaken_module.uninitAsr();
  }
}

/**************************************************
*************   Ros TTS Module  *******************
***************************************************/

TTSModuleRos::TTSModuleRos()
{
}
TTSModuleRos::~TTSModuleRos()
{
}
bool TTSModuleRos::init()
{
  requestServices();
  responseServices();
  // 获取参数
  tts_nodehandle.param("/tts_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  tts_nodehandle.param("/tts_sample/audio_save_path", audio_save_path,
                       std::string("~/catkin_ws/src/xbot_talker/cache/audio"));
  tts_nodehandle.param("/tts_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));

  tts_nodehandle.param("/tts_sample/audio_channel", audio_channel, int(1));

  // 科大讯飞asr登录以及语法参数配置
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);

  ROS_INFO_STREAM("--------TextToSpeech module is ready to start.--------");
}

void TTSModuleRos::responseServices()
{
  tts_server = tts_nodehandle.advertiseService(std::string("/xbot/xbot_tts"), &TTSModuleRos::ttsCallback, this);
  play_server = tts_nodehandle.advertiseService(std::string("/xbot/play"), &TTSModuleRos::playCallback, this);

}

bool TTSModuleRos::playCallback(xbot_talker::play::Request &req, xbot_talker::play::Response &res){
  if (req.mode == 1){
    std::string audiofile = req.audio_path;
    std::string dialogue_prompt_wav = "play " + audiofile;
    system(dialogue_prompt_wav.c_str());
  }
  if (req.mode == 2){
    ROS_INFO("----------start to request the tts service-----------");

    TextToSpeech tts_test;
    string text = req.tts_text;
    tts_test.audioConverter(log_path, text.c_str());
    ROS_INFO("tts  done.");
  }
  res.success = true;
  return true;
}


void TTSModuleRos::requestServices()
{
  tts_client = tts_nodehandle.serviceClient<xbot_talker::xbot_tts>(std::string("xbot/xbot_tts"));
}

// xbot_tts 服务的回调函数
bool TTSModuleRos::ttsCallback(xbot_talker::xbot_tts::Request& req, xbot_talker::xbot_tts::Response& res)
{
  if (req.start_tts)
  {
    ROS_INFO("----------start to request the tts service-----------");

    TextToSpeech tts_test;
    string text = req.tts_text;
    tts_test.audioConverter(log_path, text.c_str());
    ROS_INFO("tts  done.");
    res.success = true;

    return true;
  }
}
