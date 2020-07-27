#include "asr/xunfei/xfei_speech_recog.h"
#include "asr/xunfei/BuildGrammar.h"
#include "common_config.h"
#include <iostream>
#include <thread>
// xunfei asr module
// base_path表示到xbot_talker的绝对或者相对路径.
const std::string base_path = "../../..";
const int ONE_CHANNEL = 1;  // 单声道
const int TWO_CHANNEL = 2;  // 双声道测试只能用于读取已有pcm文件识别

typedef enum ASR_MODULE {
  ASR_ONLINE = 0,
  ASR_OFFLINE,
} ASR_MODULE;

typedef enum ASR_MODE {
  READ_PCM_FILE_TO_ASR = 0,
  RECORDE_TO_ASR,
} ASR_MODE;

void usage(int& asr_module, int& asr_mode)
{
  std::cout << "---------  xunfei asr demo  ------------" << std::endl;
  std::cout << "这是科大讯飞在线语音识别功能/离线语法识别的示例demo." << std::endl;
  std::cout << "你可以选择在线语音识别功能或者离线语法识别功能." << std::endl;
  std::cout << "0:在线语音识别功能;" << std::endl;
  std::cout << "1:离线语法识别功能." << std::endl;
  std::cout << "请输入0或1选择不同功能:" << std::endl;
  std::cout << "asr_module :_\b";
  std::cin >> asr_module;
  std::cout << "------------------------------" << std::endl;
  std::cout << "你可以选择通过已录好的pcm文件或者实时录音进行语音识别测试." << std::endl;
  std::cout << "0:选择已录好的pcm音频文件进行唤醒测试;" << std::endl;
  std::cout << "1:通过实时录音进行测试." << std::endl;
  std::cout << "请输入0或1选择不同方式:" << std::endl;
  std::cout << "asr_mode :_\b";
  std::cin >> asr_mode;
}

// 在线语音识别:选择已录好的pcm音频文件进行ASR
void fileToASR(const std::string asr_param, const std::string pcm_file)
{
  XfeiSpeechRecog xfei_sr_online;
  // 保存语音识别结果
  std::string xfei_online_result = "none_result";
  xfei_sr_online.setAsrParams(base_path, pcm_file, asr_param, ONE_CHANNEL);
  // 科大讯飞识别模块的初始化
  xfei_sr_online.initAsr();
  // 获取pcm_file的数据
  xfei_sr_online.getPcmFileData();
  // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的识别结果
  xfei_online_result = xfei_sr_online.dataLoopRecog();
  std::cout << "在线语音识别结果:" << xfei_online_result << std::endl;
  // 一次识别结束后释放资源
  xfei_sr_online.uninitAsr();
}

// 在线语音识别:通过实时录音进行测试ASR
void recordToASR(const std::string asr_param, const std::string pcm_file)
{
  // pcm数据的内容和数据大小
  struct DataBuff pcm_buff = { NULL, 0 };
  XfeiSpeechRecog xfei_sr_online;
  // 保存语音识别结果
  char* xfei_online_result = NULL;
  xfei_sr_online.setAsrParams(base_path, pcm_file, asr_param, ONE_CHANNEL);
  // 科大讯飞识别模块的初始化
  xfei_sr_online.initAsr();
  // record_time 录音时长(s),可修改
  float record_time = 3.5;
  // enable_record_save是否将录音保存到pcm文件
  bool enable_record_save = true;
  pcm_buff = xfei_sr_online.recordThroughMIC(record_time, enable_record_save);
  xfei_sr_online.getPCMData(pcm_buff);
  // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的识别结果
  xfei_online_result = xfei_sr_online.dataLoopRecog();
  std::cout << "在线语音识别结果:" << xfei_online_result << std::endl;
  // 一次识别结束后释放资源
  xfei_sr_online.uninitAsr();
}

// 离线语音识别:选择已录好的pcm音频文件进行ASR
void fileToASROffline(const std::string asr_param, const std::string pcm_file)
{
  XfeiSpeechRecog xfei_sr_offline;
  // 保存语音识别结果json
  char* recog_result_json = NULL;
  xfei_sr_offline.setAsrParams(base_path, pcm_file, asr_param, ONE_CHANNEL);
  // 科大讯飞识别模块的初始化
  xfei_sr_offline.initAsr();
  // 获取pcm_file的数据
  xfei_sr_offline.getPcmFileData();
  // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
  recog_result_json = xfei_sr_offline.dataLoopRecog();
  // 保存解析出的识别结果和置信度
  std::vector<std::string> offline_result_vector(2);
  if (recog_result_json == NULL)
  {
    offline_result_vector[0] = "none_result";
    offline_result_vector[1] = "0";
  }
  else
  {
    // 在识别结果不为空时，从json结果中解析出需要的结果和置信度
    offline_result_vector = xfei_sr_offline.resultFromJson();
  }
  std::cout << "离线语法识别结果:" << offline_result_vector[0] << " 置信度:" << offline_result_vector[1] << std::endl;
  // 一次识别结束后释放资源
  xfei_sr_offline.uninitAsr();
}

// 离线语音识别:通过实时录音进行测试ASR
void recordToASROffline(const std::string asr_param, const std::string pcm_file)
{
  // pcm数据的内容和数据大小
  struct DataBuff pcm_buff = { NULL, 0 };
  XfeiSpeechRecog xfei_sr_offline;
  // 保存语音识别结果json
  char* recog_result_json = NULL;
  xfei_sr_offline.setAsrParams(base_path, pcm_file, asr_param, ONE_CHANNEL);
  // 科大讯飞识别模块的初始化
  xfei_sr_offline.initAsr();
  // record_time 录音时长(s),可修改
  float record_time = 3.5;
  // enable_record_save是否将录音保存到pcm文件
  bool enable_record_save = true;
  // 固定时长录音
  pcm_buff = xfei_sr_offline.recordThroughMIC(record_time, enable_record_save);
  xfei_sr_offline.getPCMData(pcm_buff);
  // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
  recog_result_json = xfei_sr_offline.dataLoopRecog();
  // 保存解析出的识别结果和置信度
  std::vector<std::string> offline_result_vector(2);
  if (recog_result_json == NULL)
  {
    offline_result_vector[0] = "none_result";
    offline_result_vector[1] = "0";
  }
  else
  {
    // 在识别结果不为空时，从json结果中解析出需要的结果和置信度
    offline_result_vector = xfei_sr_offline.resultFromJson();
  }
  std::cout << "离线语法识别结果:" << offline_result_vector[0] << " 置信度:" << offline_result_vector[1] << std::endl;
  // 一次识别结束后释放资源
  xfei_sr_offline.uninitAsr();
}

// 科大讯飞离线语法识别构建语法
void build_grammar()
{
  // 生成的语法文件储存路径，与用户自定义文件存放在同意文件夹下
  std::string grammar_file = base_path + "/cache/grammar_config";
  // 使用制定路径存放 grammar.bnf 文件（默认存放在 xbot_talker/cache/grammar_config/grammar）
  std::string grammar_build_path = grammar_file + "/grammar";
  grammar_file = grammar_file + "/grammar.bnf";
  BuildGrammar build_grammar;
  UserData asr_data;
  // 讯飞语音sdk参数配置
  //这个路径前必须加一个fo|，否则就会语法构建不通过
  string asr_res_path = "fo|res/asr/common.jet";
  // 调用 build_grammar 函数，生成语法文件
  build_grammar.buildGrammar(&asr_data, base_path, grammar_build_path, asr_res_path, grammar_file);
}

int main(int argc, char** argv)
{
  int asr_module;
  int asr_mode;
  usage(asr_module, asr_mode);
  // pcm_file表示用于用于作为输入的音频文件,默认为xbot_talker/defaultconfig/audio/nihao_test.pcm.
  const std::string default_pcm_file = base_path + "/defaultconfig/audio/nihao_test.pcm";
  // 科大讯飞asr登录
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);

  if (asr_module == ASR_ONLINE)
  { /*******************  在线语音识别功能 ********************/

    // 设置科大讯飞在线语音识别相关参数
    std::string xfei_online_asr_params_ = "sub = iat, domain = iat, language = zh_cn, "
                                          "accent = mandarin, sample_rate = 16000, "
                                          "result_type = plain, result_encoding = utf8";

    if (asr_mode == READ_PCM_FILE_TO_ASR)
    {
      // 选择已录好的pcm音频文件进行测试
      std::string pcm_file;
      std::cout << "------------------------------" << std::endl;
      std::cout << "请输入用于测试的pcm文件,若输入为d，则用默认的文件进行测试:";
      std::cin >> pcm_file;
      if (pcm_file == "d")
      {
        pcm_file = default_pcm_file;
      }
      fileToASR(xfei_online_asr_params_, pcm_file);
    }
    else if (asr_mode == RECORDE_TO_ASR)
    {  // 通过实时录音进行测试
      recordToASR(xfei_online_asr_params_, default_pcm_file);
    }
  }
  else if (asr_module == ASR_OFFLINE)
  {
    /*******************  离线语法识别功能 ********************/
    // 构建语法
    build_grammar();
    // 语法参数配置
    std::string grammar_path = base_path + "/cache/grammar_config";
    std::string asr_params = xunfei_config.configGramParas(base_path, grammar_path);

    if (asr_mode == READ_PCM_FILE_TO_ASR)
    {
      // 选择已录好的pcm音频文件进行测试
      std::string pcm_file;
      std::cout << "------------------------------" << std::endl;
      std::cout << "请输入用于测试的pcm文件,若输入为d，则用默认的文件进行测试:";
      std::cin >> pcm_file;
      if (pcm_file == "d")
      {
        pcm_file = base_path + "/defaultconfig/audio/nihao_test.pcm";
      }
      fileToASROffline(asr_params, pcm_file);
    }
    else if (asr_mode == RECORDE_TO_ASR)
    {
      //通过实时录音进行测试
      recordToASROffline(asr_params, default_pcm_file);
    }
  }
  else
  {
    std::cout << "无效的功能模式的选择!!!" << std::endl;
    exit(1);
  }
}
