#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "nlp/nlp_feedback.h"

int main(int argc, char* argv[])
{
  TuLingRobot tuling;
  std::string ask_str;
  std::cout << "请输入你的问题: ";
  std::cin >> ask_str;
  // 设置图灵机器人对话请求参数
  tuling.setAskJson(ask_str);
  // 请求图灵机器人接口，获取回答
  std::string answer_json = tuling.callTulingApi();
  std::cout << "回答: " << answer_json << std::endl;
  // 从图灵机器人的完整json回答中解析出最终需要的text回答
  std::string tuling_answer_text = tuling.textFromJson();
  std::cout << "TuLing robot answer is:" << tuling_answer_text << std::endl;
  // 一次回答后释放资源
  tuling.uninitTuling();
  return 0;
}
