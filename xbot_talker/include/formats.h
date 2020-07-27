#ifndef FORMATS_H_160601_TT
#define FORMATS_H_160601_TT 1

#ifndef WAVE_FORMAT_PCM
#define WAVE_FORMAT_PCM 1

// 对于不超过两个通道且具有8位或16位采样的PCM音频数据，使用WAVEFORMATEX结构指定数据格式。
typedef struct tWAVEFORMATEX
{
  unsigned short wFormatTag;      // 指定格式类型; 默认 WAVE_FORMAT_PCM = 1
  unsigned short nChannels;       // 指出波形数据的声道数; 单声道为 1, 立体声为 2
  unsigned int nSamplesPerSec;    //指定采样频率(每秒的样本数)
  unsigned int nAvgBytesPerSec;   //指定数据传输的传输速率(每秒的字节数)
  unsigned short nBlockAlign;     //指定块对齐块对齐是数据的最小单位
  unsigned short wBitsPerSample;  //每个样本的位数。表示每个声道中各个样本的数据位数
  unsigned short cbSize;          //该结构（类）的大小
} WAVEFORMATEX;
#endif

#endif
