#include <stdlib.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include "asr/xunfei/BuildGrammar.h"
#include "common_config.h"
#include "asr/xunfei/msp_cmn.h"
#include "asr/xunfei/msp_errors.h"
#include "asr/xunfei/qisr.h"

using std::endl;
using std::string;

// 构建离线语法，生成相应文件。构建的语法文件存储在 /config 目录下
int main(int argc, const char** argv)
{
  // 此处使用 roslaunch 启动参数为 node path; arg1; arg2; arg3; __name; __log;
  // 使用 rosrun 启动参数为四个
  if (argc != 3 && argc != 5)
  {
    std::cout << "Wrong number of parameters!" << std::endl;
    exit(ASR_ERROR_WRONG_NUMBER_OF_PARAMS);
  }
  string base_path(argv[1]);
  // 生成的语法文件储存路径，与用户自定义文件存放在同意文件夹下
  string grammar_file(argv[2]);
  // 使用制定路径存放 grammar.bnf 文件（默认存放在 xbot_talker/cache/grammar_config/grammar 中）
  string grammar_build_path = grammar_file + "/grammar";
  grammar_file = grammar_file + "/grammar.bnf";
  BuildGrammar build_grammar;
  UserData asr_data;
  // 讯飞语音sdk参数配置
  //这个路径前必须加一个fo|，否则就会语法构建不通过
  string asr_res_path = "fo|res/asr/common.jet";
  // 讯飞登录
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);

  // 调用 build_grammar 函数，生成语法文件
  build_grammar.buildGrammar(&asr_data, base_path, grammar_build_path, asr_res_path, grammar_file);

  return 0;
}
