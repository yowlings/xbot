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

BuildGrammar::BuildGrammar()
{
}

BuildGrammar::~BuildGrammar()
{
}

// 构建离线识别语法
void BuildGrammar::buildGrammar(UserData* udata, const string base_path, const string grammar_build_path,
                                const string asr_res_path, const string grammar_file)
{
  using std::ifstream;
  using std::ios;
  using std::to_string;

  memset(udata, 0, sizeof(UserData));
  std::cout << "Start building offline grammar for recognition ..." << std::endl;
  std::cout << "Your grammar file is read from: " << grammar_file.c_str() << std::endl;
  std::cout << "Your asr grammar file will save at: " << grammar_build_path.c_str() << std::endl;
  // 将语法文件 grammar.bnf 内容读入 char* grm_content 中，并存储文件大小
  char* grm_content;
  int grm_cnt_len;
  std::ifstream infile(grammar_file.c_str());
  if (infile.is_open())
  {
    infile.seekg(0, std::ios::end);
    grm_cnt_len = infile.tellg();
    infile.seekg(0, std::ios::beg);
    grm_content = new char[grm_cnt_len];
    infile.read(grm_content, grm_cnt_len);
    infile.close();
  }
  else
  {
    std::cout << "Fail to open expected Json file" << grammar_file << endl;
    exit(ASR_ERROR_FILE_OPEN_FAIL);
  }

  // 构建 grm_build_param 参数
  string tmp = string("engine_type = local, asr_res_path = " + asr_res_path + ", sample_rate = " +
                      to_string(SAMPLE_RATE_16K) + ", grm_build_path = " + grammar_build_path + ", ");
  const char* grm_build_params = tmp.c_str();
  std::cout << grm_build_params << std::endl;

  // QISRBuildGrammar 构建语法，生成语法ID。
  // 返回: 构建成功返回0 否则返回错误码
  // 参数:
  //    grammarType[in] - 语法类型，在线识别采用 abnf 格式，离线识别采用 bnf
  //    格式。
  //    grammarContent[in] - 语法内容。
  //    grammarLength[in] - 语法长度。
  //    params[in] - 传入的参数
  //      http://mscdoc.xfyun.cn/windows/api/iFlytekMSCReferenceManual/qisr_8h.html#a1895f14ba0cfec5b9504890100c41652
  //    callback[in] - 构建语法回调接口。
  //                  typedef int ( GrammarCallBack)( int errorCode,
  //                                const char info, void* userData);
  //    userData[in/out] - 用户数据。
  int ret_config =
      QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, BuildGrammar::buildGrammarCallback, udata);

  // 判断
  if (MSP_SUCCESS != ret_config)
  {
    std::cout << "Building grammar failed!" << std::endl;
    exit(ASR_ERROR_XFEI_BUILD_GRAMMAR_FAIL);
  }
  while (BUILD_FINISH != udata->build_fini)
  {
    usleep(300 * 1000);
  }
  if (MSP_SUCCESS != udata->errcode)
  {
    exit(MSP_SUCCESS);
  }
  // save ret_config into a file
  string filepath = grammar_build_path + "/info.txt";
  writeIntoFile(filepath, udata);
  std::cout << "Build grammar finished!" << std::endl;
}

// 构建离线识别语法的回调函数
int BuildGrammar::buildGrammarCallback(int ecode, const char* info, void* udata)
{
  UserData* grm_data = (UserData*)udata;
  if (NULL != grm_data)
  {
    grm_data->build_fini = 1;
    grm_data->errcode = ecode;
  }

  if (MSP_SUCCESS == ecode && NULL != info)
  {
    std::cout << "Build grammar success! Grammar ID: " << info << std::endl;
    if (NULL != grm_data)
    {
      strcpy(grm_data->grammar_id, info);
    }
  }
  else
  {
    std::cout << "Build grammar failed!  Error code:" << ecode << std::endl;
  }
  return 0;
}

int BuildGrammar::writeIntoFile(const string file_path, UserData* info)
{
  using std::ofstream;

  std::ofstream outfile(file_path);

  if (outfile.is_open())
  {
    outfile << info->build_fini << '\n';
    outfile << info->update_fini << '\n';
    outfile << info->errcode << '\n';
    outfile << info->grammar_id << '\n';
    outfile.close();
  }
  else
  {
    std::cout << "Fail to open expected Json file" << file_path << std::endl;
    exit(ASR_ERROR_FILE_OPEN_FAIL);
  }

  return 0;
}
