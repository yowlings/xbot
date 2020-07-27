/**
 * @file    nlp_feedback.h
 * @brief   语音识别结果处理与机器人交互模块.
 * TODO: 还需要添加版权、版本等信息
 * @author  Xiaoyun Wang.
 */
#ifndef RESULT_FEEDVACK_HPP
#define RESULT_FEEDVACK_HPP
#include <curl/curl.h>
#include <fstream>
#include <iostream>
#include <vector>

/** 要执行的机器人动作,行为字典 */
enum RobotAciton
{
  NO_ACTION = 0,
  CLOSE_CHAT,
  BACK_TO_ORIGINAL_LOCATION,
  TAKE_A_STEP_FORWARD,
  TAKE_A_STEP_BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  RAISE_LEFT_HAND,
  PUT_DOWM_LEFT_HAND,
  RAISE_RIGHT_HAND,
  PUT_DOWN_RIGHT_HAND,
  LEFT_HAND_GRIP,
  LEFT_HAND_OPEN,
  RIGHT_HAND_GRIP,
  RIGHT_HAND_OPEN,
  NAVI_TO_FANGZONG,
  NAVI_TO_CHANGZONG,
  START_CHAT,
};

/** 动作模式字典 */
enum ActionMode
{
  PLAY_BEFORE_ACTION = 0,
  PLAY_AFTER_ACTION,
  PLAY_AND_ACTION_SYNC,
  PLAY_ONLY,
  ACTION_ONLY,

};

/** 语音识别结果处理 */
class ResultFeedback
{
public:
  ResultFeedback() : answer_dictionary_(4)
  {
  }

  /**
   * @fn resultIsValid
   * @brief	判断语音识别结果是否为有效结果,若语音识别模块传来的结果！=none_result,则为有效.
   * @param recog_result			- [in] 语音识别结果.
   * @return bool             - 若语音识别模块传来的结果！=none_result,则则返回结果为true.
   */
  bool resultIsValid(const std::string recog_result);

  /**
   * @fn readAnswerTable
   * @brief	读取离线对话库：defaultconfig/answer_dic.csv文件里的内容，存入answer_table_.
   * @param answer_config_file			- [in] 离线对话配置文件.
   * @return std::vector<std::vector<std::string>>  answer_table_    - 返回存储对话信息的vector.
   */
  std::vector<std::vector<std::string>> readAnswerTable(std::string answer_config_file);

  /**
   * @fn keywordDetection
   * @brief	根据交互模式answer_table_判断识别出的关键词是否有对应的响应策略.
   *        若该关键词有对应的响应回复,存入响应向量answer_dictionary_(4).
   * @return bool       - 若该关键词有对应的响应回复，返回true.
   */
  bool keywordDetection();

  /**
   * @fn keywordResponse
   * @brief	根据响应vector内的值调用不同的响应接口，如发布控制指令，播放音频文件等.
   * @param log_path			- [in] 语音合成的音频文件的缓存路径.
   */
  void keywordResponse(const std::string log_path);

  /**
   * @fn answerContent
   * @brief	返回一个反馈内容（包含语音和动作的反馈）.
   * @return std::vector<std::string> answer_dictionary_     - 返回answer_dictionary_
   */
  std::vector<std::string> answerContent();

  /**
   * @fn actionDetect
   * @brief	监测是否需要机器人控制以及控制类型..
   * @return int     - 返回RobotAciton.
   */
  int actionDetect();

  /**
   * @fn isKeywordRepeated
   * @brief	判断新加的关键词是否已被定义过.
   * @return bool     - 若已被定义过，则返回true.
   */
  bool isKeywordRepeated(std::string key_word);

  /**
   * @fn uninitNLP
   * @brief	一次处理结束后释放资源.
   */
  void uninitNLP();

private:
  std::vector<std::string> answer_dictionary_;          // 存储一个关键词及其对应的反馈内容
  std::vector<std::vector<std::string>> answer_table_;  // 存储整个csv表格的内容
  int detect_count_ = 0;                                // 存储识别到的关键词存在的回复的个数
  std::string recog_final_result_;                      // 从json串中解析出的语音识别结果
  std::string base_path_;
  std::string tts_path_;
  void robotPlay();
  int robotAction();
};

/** 图灵对话机器人模块 */
class TuLingRobot
{
public:
  /**
   * @fn setAskJson
   * @brief	设置图灵机器人对话请求参数.
   * @param ask_str			- [in] 提问的语句.
   */
  void setAskJson(const std::string ask_str);

  /**
   * @fn callTulingApi
   * @brief	请求图灵机器人接口，获取回答.
   * @return std::string answer_json_			- 返回图灵机器人的完整json回答.
   */
  std::string callTulingApi();

  /**
   * @fn textFromJson
   * @brief	从图灵机器人的完整json回答中解析出最终需要的text回答.
   * @return std::string answer_text_			- 返回需要的text回答.
   */
  std::string textFromJson();

  /**
   * @fn uninitTuling
   * @brief	一次回答后释放资源.
   */
  void uninitTuling();

private:
  std::string tuling_url_ = "http://openapi.tuling123.com/openapi/api/v2";
  std::string tuling_key_ = "58dcf416803a401a8b0770b9c551acef";
  std::string ask_json_;
  std::string answer_json_;
  std::string answer_text_;
  static size_t http_data_writer(void* data, size_t size, size_t nmemb, void* content);
};

#endif
