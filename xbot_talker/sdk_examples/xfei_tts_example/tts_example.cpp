#include "tts/text_to_speech.h"
#include "common_config.h"
#include <iostream>
#include <thread>
// base_path表示到xbot_talker的绝对或者相对路径.
const std::string base_path = "../../..";

int main(int argc, char** argv)
{
  std::cout << "---------  xunfei tts demo  ------------" << std::endl;
  std::cout << "这是科大讯飞在线语音识别功能/离线语法识别的示例demo." << std::endl;
  // 科大讯飞asr登录以及语法参数配置
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);
  TextToSpeech tts_test;
  string text;
  std::cout << " 输入想要进行语音合成的文字:";
  std::cin >> text;
  // log_path:合成的音频保存路径
  std::string log_path = base_path + "/cache/log";
  tts_test.audioConverter(log_path, text.c_str());
}
