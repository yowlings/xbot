#ifndef TEXT_TO_SPEECH_H_
#define TEXT_TO_SPEECH_H_

#include <map>
#include <string>
#include "asr/xunfei/msp_cmn.h"
#include "asr/xunfei/msp_errors.h"
#include "asr/xunfei/qtts.h"

using std::string;
using std::map;

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
  char riff[4];               // = "RIFF"
  int size_8;                 // = FileSize - 8
  char wave[4];               // = "WAVE"
  char fmt[4];                // = "fmt "
  int fmt_size;               // = 下一个结构体的大小 : 16
  short int format_tag;       // = PCM : 1
  short int channels;         // = 通道数 : 1
  int samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
  int avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample /
                              // 8
  short int block_align;      // = 每采样点字节数 : wBitsPerSample / 8
  short int bits_per_sample;  // = 量化比特数: 8 | 16
  char data[4];               // = "data";
  int data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

///* 默认wav音频头部数据 */
static wave_pcm_hdr default_wav_hdr = {
  { 'R', 'I', 'F', 'F' }, 0, { 'W', 'A', 'V', 'E' }, { 'f', 'm', 't', ' ' }, 16, 1, 1, 44100, 88200, 2, 16,
  { 'd', 'a', 't', 'a' }, 0
};

class TextToSpeech
{
private:
  //读入文本的路径
  string basePath;
  const char* src_text;
  string audioFile;
  static map<string, string> text_audio_map;

public:
  TextToSpeech(const string& basePath, const char* src_text, const string& audioFile);
  TextToSpeech();
  TextToSpeech(const char* src_text);
  static map<string, string> createMap();
  ~TextToSpeech();
  bool audioConverter(const std::string base_path, const char* src_text);
  string getAudioFile(map<string, string> current_map);
  int set_pcm_play(FILE* fp, wave_pcm_hdr* wav_header);
  int play_wav(const string& file_path);
};

#endif
