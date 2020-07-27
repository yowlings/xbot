#include "asr/xunfei/msp_cmn.h"
#include "asr/xunfei/msp_errors.h"
#include "asr/xunfei/qisr.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using std::endl;
using std::ifstream;
using std::ios;
using std::ofstream;
using std::string;
using std::to_string;

static const int FRAME_LEN = 640;
static const int BUFFER_SIZE = 4096;
static const int SAMPLE_RATE_44K = 44100;
static const int SAMPLE_RATE_16K = 16000;
static const int SAMPLE_RATE_8K = 8000;
static const int MAX_GRAMMARID_LEN = 32;
static const int MAX_PARAMS_LEN = 1024;
static const int BUILD_FINISH = 1;

// 自定义的结构体，保存语法网络构建相关的信息
// 结构体实例化为asr_data，之后都是将asr_data当作函数参数进行传递
// 在buildGrammar构建语法网络、buildGrammarCallback离线识别语法的回调函数时调用
typedef struct _UserData
{
  int build_fini;                      //标识语法构建是否完成
  int update_fini;                     //标识更新词典是否完成
  int errcode;                         //记录语法构建或更新词典回调错误码
  char grammar_id[MAX_GRAMMARID_LEN];  //保存语法构建返回的语法ID
} UserData;

class BuildGrammar
{
private:
  std::string base_path;
  std::string grammar_build_path;
  std::string asr_res_path;
  std::string grammar_file;
  UserData asr_data;
  static int buildGrammarCallback(int ecode, const char* info, void* udata);
  static int writeIntoFile(const string file_path, UserData* info);

public:
  BuildGrammar();
  ~BuildGrammar();
  void buildGrammar(UserData* udata, const string base_path, const string grammar_build_path, const string asr_res_path,
                    const string grammar_file);
};
