#include "asr/baidu/asr_online.h"
#include <iostream>
#include <thread>
// base_path表示到xbot_talker的绝对或者相对路径.
const std::string base_path = "../../..";
const int ONE_CHANNEL = 1;  // 单声道
const int TWO_CHANNEL = 2;  // 双声道测试只能用于读取已有pcm文件识别

typedef enum ASR_MODE {
  READ_PCM_FILE_TO_ASR = 0,
  RECORDE_TO_ASR,

} ASR_MODE;

void usage(int& asr_mode)
{
  std::cout << "---------  baidu asr  demo  ------------" << std::endl;
  std::cout << "这是百度在线语音识别的示例demo.\n你可以选择通过已录好的pcm文件或者实时录音进行测试." << std::endl;
  std::cout << "0:选择已录好的pcm音频文件进行测试;" << std::endl;
  std::cout << "1:通过实时录音进行测试." << std::endl;
  std::cout << "请输入0或1选择不同方式:" << std::endl;
  std::cout << "asr_mode :_\b";
  std::cin >> asr_mode;
}

// 选择已录好的pcm音频文件进行ASR
void fileToASR(const std::string pcm_file)
{
  BaiduAsrOnline baidu_sr_online;
  // 设置百度在线语音识别相关参数
  baidu_sr_online.setAsrParams(base_path, pcm_file, ONE_CHANNEL);
  // 百度语音识别模块的初始化,设置asr相关参数
  baidu_sr_online.initAndConfigAsr();
  // 获取百度在线语音识别的Token
  baidu_sr_online.speechGetToken();
  // 读取pcm文件里的音频数据
  baidu_sr_online.getPcmFileData();
  char* recog_result_online;
  // 百度在线语音识别并获取识别结果
  recog_result_online = baidu_sr_online.runAsrAndRecog();
  // 从完整的json语音识别结果中解析出需要的字符串结果
  std::string baidu_online_result = baidu_sr_online.resultFromJson();
  std::cout << "Online asr result is :" << recog_result_online << std::endl;
  std::cout << "Online asr finall result is :" << baidu_online_result << std::endl;
  // 一次识别结束后释放资源
  baidu_sr_online.uninitAsr();
}

// 通过实时录音进行asr
void recordToASR(const std::string pcm_file)
{
  BaiduAsrOnline baidu_sr_online;
  // 设置百度在线语音识别相关参数
  baidu_sr_online.setAsrParams(base_path, pcm_file, ONE_CHANNEL);
  // 百度语音识别模块的初始化,设置asr相关参数
  baidu_sr_online.initAndConfigAsr();
  // 获取百度在线语音识别的Token
  baidu_sr_online.speechGetToken();
  // record_time 录音时长(s),可修改
  float record_time = 3.5;
  // enable_record_save是否将录音保存到pcm文件
  bool enable_record_save = true;
  struct DataBuff pcm_buff = { NULL, 0 };
  // 录音
  pcm_buff = baidu_sr_online.recordThroughMIC(record_time, enable_record_save);
  // 获取pcm数据
  baidu_sr_online.getPCMData(pcm_buff);
  char* recog_result_online;
  // 百度在线语音识别并获取识别结果
  recog_result_online = baidu_sr_online.runAsrAndRecog();
  // 从完整的json语音识别结果中解析出需要的字符串结果
  std::string baidu_online_result = baidu_sr_online.resultFromJson();
  std::cout << "Online asr result is :" << recog_result_online << std::endl;
  std::cout << "Online asr finall result is :" << baidu_online_result << std::endl;
  // 一次识别结束后释放资源
  baidu_sr_online.uninitAsr();
}
int main(int argc, char** argv)
{
  int asr_mode;
  usage(asr_mode);
  // pcm_file表示用于用于作为输入的音频文件,默认为xbot_talker/defaultconfig/audio/nihao_test.pcm.
  const std::string default_pcm_file = base_path + "/defaultconfig/audio/nihao_test.pcm";
  std::string pcm_file;
  if (asr_mode == READ_PCM_FILE_TO_ASR)
  {
    /***  选择已录好的pcm音频文件进行测试  ***/
    std::cout << "请输入用于测试的pcm文件,若输入为d，则用默认的文件进行测试:";
    std::cin >> pcm_file;
    if (pcm_file == "d")
    {
      pcm_file = default_pcm_file;
    }
    fileToASR(pcm_file);
  }
  else if (asr_mode == RECORDE_TO_ASR)
  {
    /*** 通过实时录音进行测试 ***/
    recordToASR(default_pcm_file);
  }
  else
  {
    std::cout << "无效的awaken_mode模式的选择!!!" << std::endl;
    exit(1);
  }
}
