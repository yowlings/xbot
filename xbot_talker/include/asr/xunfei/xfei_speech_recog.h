/**
 * @file    xfei_speech_recog.h
 * @brief   科大讯飞语音识别模块接口头文件.
 * TODO: 还需要添加版权、版本等信息
 * @author  Xiaoyun Wang.
 */
#ifndef XFEI_SPEECH_RECOG_H_
#define XFEI_SPEECH_RECOG_H_
#include <iostream>
#include "file_operation.h"
#include "linuxrec.h"

/* 科大讯飞识别状态和参数 */
struct speech_recog
{
  int ep_stat;
  int rec_stat;     /* 关键词识别状态 */
  int audio_status; /* 录音buffer的状态 */
  volatile int state;
  char* session_begin_params; /* asr各种参数 */
};

/* 科大讯飞语音识别模块 */
class XfeiSpeechRecog
{
public:
  XfeiSpeechRecog() : recog_result_vector(2)
  {
  }

  /**
   * @fn setAsrParams
   * @brief	设置语音模块需外部传入的路径等参数.
   * @param base_path      - [in] 到/xbot_talker目录的结对路径.
   * @param pcm_file       - [in] 用于进行语音识别的录音文件.
   * @param params         - [in] 科大讯飞参数
   * @param channel        - [in] 音频声道数.1表示单声道,2表示双声道.
   */
  void setAsrParams(const std::string base_path, const std::string pcm_file, const std::string params,
                    const int channel);

  /**
   * @fn initAsr
   * @brief	科大讯飞识别模块的初始化.
   */
  void initAsr();

  /**
   * @fn getPcmFileData
   * @brief 读取pcm文件里的音频数据,并将数据的内容和数据大小存入pcm_data_.
   */
  void getPcmFileData();

  /**
   * @fn getPCMData
   * @brief	获取pcm数据接口.
   * @param pcm_buff   - [in] pcm数据的内容和数据大小.
   */
  void getPCMData(struct DataBuff pcm_buff);

  /**
   * @fn recordThroughMIC
   * @brief	录音接口.
   * @param record_time                    - [in] 录音时长(s).
   * @param enable_audio_save              - [in] 是否将录音保存到pcm文件.
   * @return	struct DataBuff	pcm_data_   - 返回录音的pcm数据和大小.
   */
  struct DataBuff recordThroughMIC(const float record_time, bool enable_audio_save);

  /**
   * @fn writeAudioData
   * @brief	将pcm音频写入科大讯飞QISRAudioWrite接口.
   * @param audio_data        - [in] pcm音频数据.
   * @param audio_len         - [in] pcm音频大小.
   */
  void writeAudioData(const char* audio_data, unsigned int audio_len);

  /**
   * @fn dataLoopRecog
   * @brief	将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果.
   * @return char* rec_result_    -  完整的json识别结果,若没有识别结果，返回NULL.
   */
  char* dataLoopRecog();

  /**
   * @fn uninitAsr
   * @brief 一次识别结束后释放资源.
   */
  void uninitAsr();

  /**
   * @fn stopRecordThroughMIC
   * @brief 关闭录音设备.
   */
  void stopRecordThroughMIC();

  /**
   * @fn resultFromJson
   * @brief 从完整的json语音识别结果中解析出需要的字符串结果和置信度值.
   * @return std::vector<std::string> recog_result_vector[2]  - 返回保存识别结果和置信度值的变量.
   */
  std::vector<std::string> resultFromJson();

private:
  /**
   * @fn getRecogResultLoop
   * @brief	获取完整的json语音识别结果.
   * @return	char*	rec_result_		   - 返回语音识别的完整json结果,若没有识别结果，返回NULL.
   */
  char* getRecogResultLoop();

  /**
   * @fn getOneChannelData
   * @brief 从双声道数据中分离出单声道数据.
   * @return struct DataBuff	pcm_data_		- 返回单声道的pcm数据和大小.
   */
  struct DataBuff getOneChannelData();

  /**
   * @fn saveRecordDataToFile
   * @brief 把录音数据存入/cache/pcm/目录下的pcm文件里，以次序和时间命名.
   */
  void saveRecordDataToFile();

  int record_loops_ = 0;
  std::vector<std::string> recog_result_vector;
  const char* session_id_ = NULL;
  int audio_stat_;
  struct DataBuff pcm_data_ = { NULL, 0 };  // 存储pcm音频数据
  struct speech_recog speech_recog_;
  char* rec_result_ = NULL;  // 保存语音识别结果
  std::string final_recog_result_;
  int recog_confidence_ = 0;
  std::string base_path_;
  std::string pcm_file_;
  std::string asr_params_;
  int channel_;
  struct recorder* asr_record_ = NULL;
  FileOperation pcm_file_operation;
  RecordAlsaAPI record_alsa;
};

#endif  // XFEI_SPEECH_RECOG_H_
