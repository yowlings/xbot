#include "asr/xunfei/xfei_speech_recog.h"
#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <thread>
#include "asr/xunfei/BuildGrammar.h"
#include "common_config.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "asr/xunfei/qisr.h"

void XfeiSpeechRecog::setAsrParams(const std::string base_path, const std::string pcm_file, const std::string params,
                                   const int channel)
{
  base_path_ = base_path;
  pcm_file_ = pcm_file;
  asr_params_ = params;
  channel_ = channel;
}

void XfeiSpeechRecog::initAsr()
{
  int error_code = 0;
  session_id_ = QISRSessionBegin(NULL, (const char*)asr_params_.c_str(), &error_code);
  if (MSP_SUCCESS != error_code)
  {
    CommonConfig& xunfei_config = CommonConfig::get_instance();
    std::cout << "QISRSessionBegin failed : errorcode:" << error_code << std::endl;
  }
}

// 从用于语音唤醒的pcm音频文件中读取数据
void XfeiSpeechRecog::getPcmFileData()
{
  FileOperation pcm_file;
  pcm_data_ = pcm_file.readFileAsDatabuffer(pcm_file_);
}

void XfeiSpeechRecog::writeAudioData(const char* audio_data, unsigned int audio_len)
{
  int err_code;
  err_code = QISRAudioWrite(session_id_, (const void*)audio_data, audio_len, speech_recog_.audio_status,
                            &speech_recog_.ep_stat, &speech_recog_.rec_stat);
  if (MSP_SUCCESS != err_code)
  {
    std::cout << "QIVWAudioWrite failed! error code:" << err_code << std::endl;
    exit(ASR_ERROR_XFEI_API_FAIL);
  }
}

char* XfeiSpeechRecog::getRecogResultLoop()
{
  int ret = 0;
  const char* qisr_result = NULL;
  int rss_status = MSP_REC_STATUS_INCOMPLETE;
  while (MSP_REC_STATUS_COMPLETE != rss_status && MSP_SUCCESS == ret)
  {
    qisr_result = QISRGetResult(session_id_, &rss_status, 0, &ret);
    if (MSP_SUCCESS != ret)
    {
      if (MSP_SUCCESS != ret)
      {
        std::cout << "QISRGetResult failed ! errorcode:" << ret << std::endl;
      }
    }
    if (qisr_result != NULL)
    {
      rec_result_ = (char*)malloc(BUFFER_SIZE);
      if (rec_result_ == NULL)
      {
        std::cout << "\n Malloc failed in rec_result !!!\n";
        exit(ASR_ERROR_MALLOC_FAIL);
      }
      rec_result_ = strncpy(rec_result_, qisr_result, BUFFER_SIZE);
    }
  }
}

// 从双声道的音频数据中分离出单声道数据
struct DataBuff XfeiSpeechRecog::getOneChannelData()
{
  struct DataBuff pcm_two_channel = pcm_data_;
  pcm_data_ = { NULL, 0 };
  pcm_data_.size = pcm_two_channel.size / 2;

  pcm_data_.data = new char[pcm_data_.size];
  for (int i = 0; i < pcm_data_.size / 2; i++)
  {
    memcpy((uint16_t*)pcm_data_.data + i, ((uint32_t*)(pcm_two_channel.data)) + i, 2);
  }
  return pcm_data_;
}

char* XfeiSpeechRecog::dataLoopRecog()
{
  rec_result_ = NULL;
  long pcm_index = 0;          // pcm数据分段索引,0表示第一段音频的起始位置.
  bool is_last_audio = false;  // 是否已读取到最后一块音频的标志.
  int ret = 0;                 // 错误标志位
  int rss_status = MSP_REC_STATUS_INCOMPLETE;

  // 对iat中一些变量进行初始化.
  speech_recog_.audio_status = MSP_AUDIO_SAMPLE_CONTINUE;
  speech_recog_.ep_stat = MSP_EP_LOOKING_FOR_SPEECH;
  speech_recog_.rec_stat = MSP_REC_STATUS_INCOMPLETE;

  // 根据音频数据是单声道还是双声道选择接口
  if (channel_ == 2)
  {
    getOneChannelData();
  }

  // 循环写入pcm数据用于语音识别,当ep_stat的状态变为MSP_EP_AFTER_SPEECH表示音频输入结束
  // 以及pcm_data.size的大小小于0时,break.
  std::cout << "-----------Start Recognizing--------" << std::endl;
  while (1)
  {
    //定义音频数据长度,单位字节.一秒32k，6400为200ms的数据。
    const unsigned int default_wave_len = 6400;
    unsigned int wave_len = default_wave_len;

    //如果pcm_data.size小于6400，说明已经处理到最后一块音频.
    if (pcm_data_.size < default_wave_len)
    {
      wave_len = pcm_data_.size;
      is_last_audio = true;
    }

    speech_recog_.audio_status = MSP_AUDIO_SAMPLE_CONTINUE;
    if (0 == pcm_index)
      speech_recog_.audio_status = MSP_AUDIO_SAMPLE_FIRST;
    writeAudioData(&pcm_data_.data[pcm_index], wave_len);

    //每写入长度为wave_len的一段音频数据后,数据数组的索引加wave_len,pcm数据的长度-wave_len.
    pcm_index += wave_len;
    pcm_data_.size -= wave_len;
    // 最后一块音频数据处理结束，跳出循环.
    if (pcm_data_.size <= 0)
      break;
    if (MSP_EP_AFTER_SPEECH == speech_recog_.ep_stat)
      break;
  }
  //检测到音频结束,写入NULL空数据主动停止识别.
  speech_recog_.audio_status = MSP_AUDIO_SAMPLE_LAST;
  writeAudioData(NULL, 0);
  getRecogResultLoop();
  return rec_result_;
}
void XfeiSpeechRecog::getPCMData(struct DataBuff pcm_buff)
{
  pcm_data_ = pcm_buff;
}

// 一次对话结束后释放资源
void XfeiSpeechRecog::uninitAsr()
{
  // 结束本次语音识别
  int ret = QISRSessionEnd(session_id_, "normal end");
  if (MSP_SUCCESS != ret)
  {
    std::cout << "QISRSessionEnd failed !errorcode:" << ret << std::endl;
  }
  if (rec_result_)
  {
    free(rec_result_);
    rec_result_ = NULL;
  }
  final_recog_result_ = "";
  recog_confidence_ = 0;
  recog_result_vector[0] = "";
  recog_result_vector[1] = "";
  pcm_data_ = { NULL, 0 };
}

// 从完整的json语音识别结果解析出string类型的关键词和置信度,
// 并认为置信度低于40的识别结果属于无效结果
std::vector<std::string> XfeiSpeechRecog::resultFromJson()
{
  rapidjson::Document doc;
  doc.Parse(rec_result_);
  if (doc.HasParseError())
  {
    rapidjson::ParseErrorCode code = doc.GetParseError();
    std::cout << "JSON解析错误" << code << std::endl;
    exit(ASR_ERROR_JSON_PARSE_FAIL);
  }
  else
  {
    rapidjson::Value& vConfidence = doc["sc"];
    recog_confidence_ = vConfidence.GetInt();
    rapidjson::Value& wordArr = doc["ws"];
    for (int i = 0; i < wordArr.Size(); ++i)
    {
      rapidjson::Value& wordUnit = wordArr[i];
      if (wordUnit.HasMember("cw"))
      {
        rapidjson::Value& contentWord = wordUnit["cw"];
        rapidjson::Value& word = contentWord[0];
        final_recog_result_ = word["w"].GetString();
      }
    }
  }
  recog_result_vector[0] = final_recog_result_;
  recog_result_vector[1] = std::to_string(recog_confidence_);

  std::cout << "Speech recognition result:" << final_recog_result_ << " |confidence:" << recog_confidence_ << std::endl;

  return recog_result_vector;
}

// Fixme:测试用的，确定后记得删除
int test_vda_count = 0;
void XfeiSpeechRecog::saveRecordDataToFile()
{
  std::string save_file_name = pcm_file_operation.setFileName("-asr.pcm");
  std::string save_file = base_path_ + "/cache/pcm" + save_file_name;
  std::ofstream pcm_file(save_file, std::ofstream::binary);
  pcm_file.write(pcm_data_.data, pcm_data_.size);
  pcm_file.close();
}

struct DataBuff XfeiSpeechRecog::recordThroughMIC(const float record_time, bool enable_audio_save)
{
  int success_code = 0;
  int errorcode;
  std::cout << "-----------Start ASR Recording  Thread --------" << std::endl;
  // 采用默认设备获取音频
  record_dev_id device_id = getDefaultInputDevice();
  int errcode = 0;
  // 使用WAVEFORMATEX结构指定pcm数据格式。
  WAVEFORMATEX wavfmt = { WAVE_FORMAT_PCM, 1, 16000, 32000, 2, 16, sizeof(WAVEFORMATEX) };
  if (getInputDeviceNum() == 0)
  {
    std::cout << "\nNo active record device find! ";
  }
  else
  {
    std::cout << "The total number of active input devices is : " << getInputDeviceNum() << std::endl;
  }
  // 设置myrec用于存储录音信息.
  asr_record_ = (struct recorder*)malloc(sizeof(struct recorder));
  if (asr_record_ == NULL)
  {
    std::cout << "\n Malloc failed in asr_record_ !!!\n";
    exit(ASR_ERROR_MALLOC_FAIL);
  }

  memset(asr_record_, 0, sizeof(struct recorder));

  asr_record_->state = RECORD_STATE_CREATED;

  asr_record_->pcm_file_path = base_path_ + "/cache/pcm";
  record_alsa.initRecord(asr_record_, device_id, &wavfmt);
  record_loops_ = record_alsa.setRecordDuration(record_time);
  int buf_count = 0;  //分段录音计数
  struct DataBuff record_pcm;

  // Fixme:测试用的，确定后记得删除
  //std::ofstream outFile(base_path_ + "/cache/" + std::to_string(test_vda_count) + "test.txt", std::ios::app);
  test_vda_count++;
  VDADetection VDA_detec;
  int speech_count = 0;
  bool is_speech = false;
  bool is_speech_end = false;
  float backgrand_energy = 0;
  int end_count = 0;
  while (record_loops_ > 0)
  {
    float level_energy = 0;
    record_pcm = record_alsa.startRecord();
    if (buf_count < 15)
    {
      backgrand_energy = VDA_detec.levelEnergy(record_pcm, 15);
      //outFile << backgrand_energy << std::endl;
    }
    else
    {
      level_energy = VDA_detec.levelEnergy(record_pcm, 15);
      //outFile << level_energy << std::endl;
    }

    if ((level_energy - backgrand_energy) > 2.2)
    {
      speech_count++;
    }
    if (speech_count >= 30)
    {
      is_speech = true;
    }
    if ((buf_count >= 450) && (is_speech == true) && (buf_count <= 550))
    {
      if (level_energy < (backgrand_energy + 1))
      {
        end_count++;
      }
      if (end_count >= 70)
      {
        is_speech_end = true;
      }
    }
    if (is_speech_end == true)
    {
      std::cout << "speech done!" << std::endl;
      break;
    }
    record_loops_--;
    pcm_data_.data = (char*)realloc(pcm_data_.data, record_pcm.size * (buf_count + 1));

    if (pcm_data_.data == NULL)
    {
      std::cout << "ERROR:buf_new realloc error!" << std::endl;
      exit(ASR_ERROR_REALLOC_FAIL);
    }
    std::memcpy(&pcm_data_.data[record_pcm.size * buf_count], record_pcm.data, record_pcm.size);

    buf_count += 1;
  }
  pcm_data_.size = record_pcm.size * buf_count;
  //outFile.close();
  // 根据选项选择是否保存录下的音频数据到文件。
  if (enable_audio_save)
  {
    std::thread save_pcm_to_file(&XfeiSpeechRecog::saveRecordDataToFile, this);
    save_pcm_to_file.detach();
  }
  return pcm_data_;
}

void XfeiSpeechRecog::stopRecordThroughMIC()
{
  record_alsa.closeRecord();
}
