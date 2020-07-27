#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_
#include <iostream>
/**
 * @brief 通用错误码定义
 */
typedef enum ERROR_CODE {
  RETURN_OK = 0,
  ERROR_XUNFEI_LOGIN_FAIL,    // 科大讯飞登录失败
  ERROR_FILE_OPEN_FAIL,       // 本地文件打开失败
  ERROR_DIRECTORY_OPEN_FAIL,  // 本地文件目录打开失败

} ERROR_CODE;

/**
 * @brief 录音相关操作错误码定义
 */
typedef enum RECORD_ERROR_CODE {
  RECORD_SUCCESS = 0,
  RECORD_ERROR_GENERAL,                      // 通用返回错误
  RECORD_ERROR_INVAL,                        // 录音结构体未初始化
  RECORD_ERROR_NOT_READY,                    // 录音还没准备好
  RECORD_ERROR_OPEN_PCM_DEVICE_FAIL,         // ALSA:打开pcm设备失败
  RECORD_ERROR_CONFIG_PCM_FAIL,              // ALSA:PCM设备配置失败
  RECORD_ERROR_ACCESS_TYPE_NOT_AVAILABLE,    // ALSA:PCM设备访问类型不可用
  RECORD_ERROR_INVALID_WAV_FORMAT,           // ALSA:无效的WAV格式
  RECORD_ERROR_SAMPLE_FORMAT_NOT_AVAILABLE,  // ALSA:样本格式不可用
  RECORD_ERROR_CHANNEL_COUNT_NOT_AVAILABLE,  // ALSA:通道数不可用
  RECORD_ERROR_RATE_SET_FAIL,                // ALSA:采样率设置失败
  RECORD_ERROR_SET_PERIOD_TIME_FAIL,         // ALSA:设置时间段失败
  RECORD_ERROR_INSTALL_HW_PARAM_FAIL,        // ALSA:写入配置失败
  RECORD_ERROR_GET_PERIOD_SIZE_FAIL,         // ALSA:获取周期大小失败
  RECORD_ERROR_GET_PERIOD_TIME_FAIL,         // ALSA:获取周期时间失败
  RECORD_ERROR_MALLOC_FAIL,                  // malloc分配内存失败
  RECORD_ERROR_OVERRUN,                      // ALSA:超限
  RECORD_ERROR_FAIL_TO_READ,                 // ALSA:读取失败
  RECORD_ERROR_SHORT_READ,                   // ALSA:读取过短

} RECORD_ERROR_CODE;

/**
 * @brief ASR MODULE错误码定义
 */
typedef enum ASR_ERROR_CODE {
  ASR_RETURN_OK = 0,                   // 返回正常
  ASR_RETURN_ERROR,                    // 返回错误
  ASR_ERROR_FILE_NOT_EXIST,            // 本地文件不存在
  ASR_ERROR_FILE_OPEN_FAIL,            // 本地文件打开失败
  ASR_ERROR_REALLOC_FAIL,              // realloc分配内存失败
  ASR_ERROR_MALLOC_FAIL,               // malloc分配内存失败
  ASR_ERROR_JSON_PARSE_FAIL,           // 字符串json解析报错
  ASR_ERROR_TOKEN_CURL,                // TOKEN CURL 调用错误
  ASR_ERROR_TOKEN_PARSE_ACCESS_TOKEN,  // access_token字段在返回结果中不存在
  ASR_ERROR_TOKEN_PARSE_SCOPE,         // 解析scope字段，或者scope不存在
  ASR_ERROR_CURL,                      // 识别 curl 错误
  ASR_ERROR_XFEI_API_FAIL,  // 调用科大讯飞语音识别API时报错，需根据log输出的errorcode查看科大讯飞官方开发文档
  ASR_ERROR_WRONG_NUMBER_OF_PARAMS,  // 参数个数错误
  ASR_ERROR_XFEI_BUILD_GRAMMAR_FAIL  // 参数个数错误

} ASR_ERROR_CODE;

/**
 * @brief AWAKEN MODULE错误码定义
 */
typedef enum AWAKEN_ERROR_CODE {
  AWAKEN_RETURN_OK = 0,          // 返回正常
  AWAKEN_RETURN_ERROR,           // 返回错误
  AWAKEN_ERROR_FILE_NOT_EXIST,   // 本地文件不存在
  AWAKEN_ERROR_REALLOC_FAIL,     // realloc分配内存失败
  AWAKEN_ERROR_MALLOC_FAIL,      // malloc分配内存失败
  AWAKEN_ERROR_JSON_PARSE_FAIL,  // 字符串json解析报错
  AWAKEN_ERROR_XFEI_API_FAIL,  // 调用科大讯飞离线语音唤醒API时报错，需根据log输出的errorcode查看科大讯飞官方开发文档
} AWAKEN_ERROR_CODE;

/**
 * @brief TTS MODULE错误码定义
 */
typedef enum TTS_ERROR_CODE {
  TTS_RETURN_SUCCESS = 0,                  // 返回正常
  TTS_ERROR_OPEN_PCM_DEVICE_FAIL,          // ALSA:打开pcm设备失败
  TTS_ERROR_SND_PCM_HW_PARAM_ALLOCA_FAIL,  //  ALSA:分配params结构体失败
  TTS_ERROR_SND_PCM_HW_PARAMS_ANY,         //  ALSA:初始化params失败
  TTS_ERROR_SND_PCM_HW_SET_ACCESS,         //  ALSA:初始化访问权限失败
  TTS_ERROR_SET_CHANNEL_FAIL,              // ALSA:设置通道数失败
  TTS_ERROR_RATE_SET_FAIL,                 // ALSA:采样率设置失败
  TTS_ERROR_SET_HW_PARAM_FAIL,             // ALSA:设置参数失败
  TTS_ERROR_GET_PERIOD_SIZE_FAIL,          // ALSA:获取周期大小失败
  TTS_ERROR_MALLOC_FAIL,                   // malloc分配内存失败
  TTS_ERROR_FILE_OPEN_FAIL,                // 本地文件打开失败

} TTS_ERROR_CODE;

/**
 * @brief NLP MODULE错误码定义
 */
typedef enum NLP_ERROR_CODE {
  NLP_RETURN_SUCCESS = 0,     // 返回正常
  NLP_ERROR_JSON_PARSE_FAIL,  // 字符串json解析报错
  NLP_ERROR_CURL,             // 识别 curl 错误

} NLP_ERROR_CODE;

class CommonConfig
{
public:
  void loginToXunfei(const std::string base_path);
  std::string configGramParas(const std::string base_path, const std::string grammar_file);
  std::string handleErrorcode(const int errorcode);

public:
  ~CommonConfig()
  {
    std::cout << "class CommonConfig destructor called!" << std::endl;
  }
  CommonConfig(const CommonConfig&) = delete;
  CommonConfig& operator=(const CommonConfig&) = delete;
  static CommonConfig& get_instance()
  {
    static CommonConfig instance;
    return instance;
  }

private:
  CommonConfig()
  {
    std::cout << "class CommonConfig constructor called!" << std::endl;
  }
};

#endif
