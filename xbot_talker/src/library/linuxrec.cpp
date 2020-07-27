#include "linuxrec.h"
#include <alsa/asoundlib.h>
#include <fcntl.h>
#include <file_operation.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include "common_config.h"
#include "formats.h"
#define DBG_ON 1
#if DBG_ON
#define dbg printf
#else
#define dbg
#endif

/* Do not change the sequence */

#define SAMPLE_RATE 16000

#define SAMPLE_BIT_SIZE 16

#define FRAME_CNT 10

//#define BUF_COUNT   1

#define DEF_BUFF_TIME 500000

#define DEF_PERIOD_TIME 100000

float VDADetection::levelEnergy(struct DataBuff pcm_data, const int count_sum)
{
  call_count_++;
  bool is_speech = false;
  is_speech_count_ = 0;
  float current_energy = energyPerSample(pcm_data);
  if (call_count_ <= count_sum)
  {
    temp_energy_ += current_energy;
  }
  // 每count_sum组数据求一个平均值,根据这个值调整背景噪声能量值
  if (call_count_ == count_sum)
  {
    temp_energy_ += current_energy;
    mean_energy_ = temp_energy_ / (float)count_sum;
    temp_energy_ = 0.0;
    call_count_ = 0;
  }
  return mean_energy_;
}

float VDADetection::energyPerSample(struct DataBuff pcm_data)
{
  float energy = 0;
  for (int i = 0; i < pcm_data.size; i++)
  {
    energy += pcm_data.data[i] * pcm_data.data[i];
  }
  energy = 10 * log(energy);

  return energy;
}

static int format_ms_to_alsa(const WAVEFORMATEX* wavfmt, snd_pcm_format_t* format)
{
  snd_pcm_format_t tmp;
  tmp = snd_pcm_build_linear_format(wavfmt->wBitsPerSample, wavfmt->wBitsPerSample, wavfmt->wBitsPerSample == 8 ? 1 : 0,
                                    0);
  if (tmp == SND_PCM_FORMAT_UNKNOWN)
    return -EINVAL;
  *format = tmp;
  return 0;
}

void RecordAlsaAPI::initRecord(struct recorder* rec, record_dev_id dev, WAVEFORMATEX* wavfmt)
{
  record_ = rec;
  int error_code = 0;
  //创建一个句柄并打开与声卡号和音频设备号设备的音频接口的连接。成功返回零，失败返回负的错误码。
  error_code = snd_pcm_open((snd_pcm_t**)&record_->wavein_hdl, dev.u.name, SND_PCM_STREAM_CAPTURE, 0);
  if (error_code < 0)
  {
    std::cout << "unable to open pcm device: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_OPEN_PCM_DEVICE_FAIL);
  }
  /* 分配硬件参数对象. */
  snd_pcm_hw_params_alloca(&params_);
  handle_ = (snd_pcm_t*)record_->wavein_hdl;
  /* 用默认值填写. */
  error_code = snd_pcm_hw_params_any(handle_, params_);
  if (error_code < 0)
  {
    std::cout << "Broken configuration for this PCM: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_CONFIG_PCM_FAIL);
  }

  /* 设置所需的硬件参数. */

  error_code = snd_pcm_hw_params_set_access(handle_, params_, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (error_code < 0)
  {
    std::cout << "Access type not available: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_ACCESS_TYPE_NOT_AVAILABLE);
  }

  /* 判断定义的wave格式是否正确 */
  error_code = format_ms_to_alsa(wavfmt, &format_);
  if (error_code)
  {
    std::cout << "Invalid format";
    exit(RECORD_ERROR_INVALID_WAV_FORMAT);
  }
  error_code = snd_pcm_hw_params_set_format(handle_, params_, format_);
  if (error_code < 0)
  {
    std::cout << "Sample format not available: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_SAMPLE_FORMAT_NOT_AVAILABLE);
  }

  /* One channels (stereo) */
  error_code = snd_pcm_hw_params_set_channels(handle_, params_, wavfmt->nChannels);
  if (error_code < 0)
  {
    std::cout << "Channels count not available: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_CHANNEL_COUNT_NOT_AVAILABLE);
  }

  /* 16000 bits/second sampling rate  */
  record_rate_ = wavfmt->nSamplesPerSec;
  error_code = snd_pcm_hw_params_set_rate_near(handle_, params_, &record_rate_, 0);
  if (error_code < 0)
  {
    std::cout << "Set rate failed: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_RATE_SET_FAIL);
  }

  /* 周期长度（帧数）. */
  frames_ = 16;
  error_code = snd_pcm_hw_params_set_period_size_near(handle_, params_, &frames_, 0);
  if (error_code < 0)
  {
    std::cout << "set period time fail: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_SET_PERIOD_TIME_FAIL);
  }

  /*将配置写入驱动程序中 */
  error_code = snd_pcm_hw_params(handle_, params_);
  if (error_code < 0)
  {
    std::cout << "Unable to install hw params: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_INSTALL_HW_PARAM_FAIL);
  }

  error_code = snd_pcm_hw_params_get_period_size(params_, &frames_, 0);
  if (error_code < 0)
  {
    std::cout << "\n get_period_size failed !!!\n" << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_GET_PERIOD_SIZE_FAIL);
  }
  audiobuf_size_ = frames_ * wavfmt->nChannels * wavfmt->wBitsPerSample / 8;
  record_->audiobuf = (char*)malloc(audiobuf_size_);
  if (record_->audiobuf == NULL)
  {
    std::cout << "\n Malloc failed in rec->audiobuf !!!\n";
    exit(RECORD_ERROR_MALLOC_FAIL);
  }

  error_code = snd_pcm_hw_params_get_period_time(params_, &record_rate_, 0);
  if (error_code < 0)
  {
    std::cout << "\n get_period_time failed !!!\n" << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_GET_PERIOD_TIME_FAIL);
  }
}
// 输入duration_time:单位s
int RecordAlsaAPI::setRecordDuration(const float duration_time)
{
  duration_time_ = (int)(duration_time * pow(10, 6));
  record_loop_ = duration_time_ / record_rate_;
  return record_loop_;
}

struct DataBuff RecordAlsaAPI::startRecord()
{
  int error_code;
  audio_pcm_ = { NULL, 0 };
  error_code = snd_pcm_readi(handle_, record_->audiobuf, frames_);
  if (error_code == -EPIPE)
  {
    /* EPIPE means overrun */
    std::cout << "overrun occurred\n";
    snd_pcm_prepare(handle_);
    exit(RECORD_ERROR_OVERRUN);
  }
  else if (error_code < 0)
  {
    std::cout << "error from read: " << snd_strerror(error_code) << std::endl;
    exit(RECORD_ERROR_FAIL_TO_READ);
  }
  else if (error_code != (int)frames_)
  {
    std::cout << "short read, read " << error_code << "  frames\n";
    exit(RECORD_ERROR_SHORT_READ);
  }
  audio_pcm_.data = record_->audiobuf;
  audio_pcm_.size = audiobuf_size_;
  return audio_pcm_;
}

void RecordAlsaAPI::closeRecord()
{
  snd_pcm_drop(handle_);
  snd_pcm_drain(handle_);
  snd_pcm_close(handle_);
  handle_ = NULL;
  free(record_);
}

static int g_show_xrun = 1;
static int startRecordInternal(snd_pcm_t* pcm)
{
  return snd_pcm_start(pcm);
}

static int stopRecordInternal(snd_pcm_t* pcm)
{
  return snd_pcm_drop(pcm);
}

static int isStoppedInternal(struct recorder* rec)
{
  snd_pcm_state_t state;
  state = snd_pcm_state((snd_pcm_t*)rec->wavein_hdl);
  switch (state)
  {
    case SND_PCM_STATE_RUNNING:
    case SND_PCM_STATE_DRAINING:
      return 0;
    default:
      break;
  }
  return 1;
}

/* set hardware and software params */
static int set_hwparams(struct recorder* rec, const WAVEFORMATEX* wavfmt, unsigned int buffertime,
                        unsigned int periodtime)
{
  snd_pcm_hw_params_t* params;
  int err;
  unsigned int rate;
  snd_pcm_format_t format;
  snd_pcm_uframes_t size;
  snd_pcm_t* handle = (snd_pcm_t*)rec->wavein_hdl;
  rec->buffer_time = buffertime;
  rec->period_time = periodtime;
  snd_pcm_hw_params_alloca(&params);
  err = snd_pcm_hw_params_any(handle, params);
  if (err < 0)
  {
    dbg("Broken configuration for this PCM");
    return err;
  }

  err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

  if (err < 0)
  {
    dbg("Access type not available");
    return err;
  }

  err = format_ms_to_alsa(wavfmt, &format);

  if (err)
  {
    dbg("Invalid format");
    return -EINVAL;
  }

  err = snd_pcm_hw_params_set_format(handle, params, format);

  if (err < 0)
  {
    dbg("Sample format non available");
    return err;
  }

  err = snd_pcm_hw_params_set_channels(handle, params, wavfmt->nChannels);

  if (err < 0)
  {
    dbg("Channels count non available");
    return err;
  }

  rate = wavfmt->nSamplesPerSec;

  err = snd_pcm_hw_params_set_rate_near(handle, params, &rate, 0);

  if (err < 0)
  {
    dbg("Set rate failed");
    return err;
  }

  if (rate != wavfmt->nSamplesPerSec)
  {
    dbg("Rate mismatch");
    return -EINVAL;
  }

  if (rec->buffer_time == 0 || rec->period_time == 0)
  {
    err = snd_pcm_hw_params_get_buffer_time_max(params, &rec->buffer_time, 0);
    assert(err >= 0);

    if (rec->buffer_time > 500000)
      rec->buffer_time = 500000;
    rec->period_time = rec->buffer_time / 4;
  }

  err = snd_pcm_hw_params_set_period_time_near(handle, params, &rec->period_time, 0);
  if (err < 0)
  {
    dbg("set period time fail");
    return err;
  }

  err = snd_pcm_hw_params_set_buffer_time_near(handle, params, &rec->buffer_time, 0);
  if (err < 0)
  {
    dbg("set buffer time failed");
    return err;
  }

  err = snd_pcm_hw_params_get_period_size(params, &size, 0);
  if (err < 0)
  {
    dbg("get period size fail");
    return err;
  }

  rec->period_frames = size;

  err = snd_pcm_hw_params_get_buffer_size(params, &size);
  if (size == rec->period_frames)
  {
    dbg("Can't use period equal to buffer size (%lu == %lu)", size, rec->period_frames);
    return -EINVAL;
  }

  rec->buffer_frames = size;
  rec->bits_per_frame = wavfmt->wBitsPerSample;

  /* set to driver */
  err = snd_pcm_hw_params(handle, params);
  if (err < 0)
  {
    dbg("Unable to install hw params:");
    return err;
  }
  return 0;
}

static int set_swparams(struct recorder* rec)
{
  int err;
  snd_pcm_sw_params_t* swparams;
  snd_pcm_t* handle = (snd_pcm_t*)(rec->wavein_hdl);

  /* sw para */
  snd_pcm_sw_params_alloca(&swparams);
  err = snd_pcm_sw_params_current(handle, swparams);
  if (err < 0)
  {
    dbg("get current sw para fail");
    return err;
  }

  err = snd_pcm_sw_params_set_avail_min(handle, swparams, rec->period_frames);
  if (err < 0)
  {
    dbg("set avail min failed");
    return err;
  }

  /* set a value bigger than the buffer frames to prevent the auto start.
   * we use the snd_pcm_start to explicit start the pcm */

  err = snd_pcm_sw_params_set_start_threshold(handle, swparams, rec->buffer_frames * 2);

  if (err < 0)
  {
    dbg("set start threshold fail");
    return err;
  }

  if ((err = snd_pcm_sw_params(handle, swparams)) < 0)
  {
    dbg("unable to install sw params:");
    return err;
  }

  return 0;
}

static int set_params(struct recorder* rec, WAVEFORMATEX* fmt, unsigned int buffertime, unsigned int periodtime)
{
  int err;
  WAVEFORMATEX defmt = { WAVE_FORMAT_PCM, 1, 16000, 32000, 2, 16, sizeof(WAVEFORMATEX) };

  if (fmt == NULL)
  {
    fmt = &defmt;
  }

  err = set_hwparams(rec, fmt, buffertime, periodtime);

  if (err)
    return err;

  err = set_swparams(rec);

  if (err)
    return err;

  return 0;
}

/*

 *   Underrun and suspend recovery

 */

static int xrun_recovery(snd_pcm_t* handle, int err)
{
  if (err == -EPIPE)
  { /* over-run */
    if (g_show_xrun)
      dbg("!!!!!!overrun happend!!!!!!");
    err = snd_pcm_prepare(handle);
    if (err < 0)
    {
      if (g_show_xrun)
        dbg("Can't recovery from overrun,"
            "prepare failed: %s\n",
            snd_strerror(err));
      return err;
    }
    return 0;
  }
  else if (err == -ESTRPIPE)
  {
    while ((err = snd_pcm_resume(handle)) == -EAGAIN)
      usleep(200000); /* wait until the suspend flag is released */
    if (err < 0)
    {
      err = snd_pcm_prepare(handle);
      if (err < 0)
      {
        if (g_show_xrun)
          dbg("Can't recovery from suspend,"
              "prepare failed: %s\n",
              snd_strerror(err));
        return err;
      }
    }
    return 0;
  }
  return err;
}

static ssize_t pcm_read(struct recorder* rec, size_t rcount)
{
  ssize_t r;
  size_t count = rcount;
  char* data;

  snd_pcm_t* handle = (snd_pcm_t*)rec->wavein_hdl;

  if (!handle)
    return -EINVAL;

  data = rec->audiobuf;

  while (count > 0)
  {
    r = snd_pcm_readi(handle, data, count);
    if (r == -EAGAIN || (r >= 0 && (size_t)r < count))
    {
      snd_pcm_wait(handle, 100);
    }
    else if (r < 0)
    {
      if (xrun_recovery(handle, r) < 0)
      {
        return -1;
      }
    }
    if (r > 0)
    {
      count -= r;
      data += r * rec->bits_per_frame / 8;
    }
  }

  return rcount;
}

static void* recordThreadProc(void* para)
{
  struct recorder* rec = (struct recorder*)para;
  size_t frames, bytes;
  sigset_t mask, oldmask;

  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

  // 当语音识别结束需要释放资源关闭录音设备的时候（closeRecorder()），跳出循环。
  while (true)
  {
    frames = rec->period_frames;
    bytes = frames * rec->bits_per_frame / 8;
    /* closing, exit the thread */
    if (rec->state == RECORD_STATE_CLOSING)
    {
      std::cout << "RECORD_STATE_CLOSING and break!!!\n";
      break;
    }
    if (rec->state < RECORD_STATE_RECORDING)
    {
      std::cout << "rec->state < RECORD_STATE_RECORDING and usleep!!!\n";
      usleep(100000);
    }
    if (pcm_read(rec, frames) != frames)
    {
      return NULL;
    }
    if (rec->on_data_ind)
    {
      rec->on_data_ind(rec->audiobuf, bytes, rec->user_cb_para);
    }
  }
  return rec;
}

static int createRecordThread(void* para, pthread_t* tidp, std::string base_path)
{
  int err;
  struct recorder* rec = (struct recorder*)para;
  rec->pcm_file_path = base_path;
  err = pthread_create(tidp, NULL, recordThreadProc, (void*)rec);

  if (err != 0)
    return err;
  return 0;
}

static void free_rec_buffer(struct recorder* rec)
{
  if (rec->audiobuf)
  {
    free(rec->audiobuf);
    rec->audiobuf = NULL;
  }
}

static int prepareRecBuffer(struct recorder* rec)
{
  /* the read and QISRWrite is blocked, currently only support one buffer,
   * if overrun too much, need more buffer and another new thread
   * to write the audio to network */
  size_t sz = (rec->period_frames * rec->bits_per_frame / 8);
  rec->audiobuf = (char*)malloc(sz);
  if (!rec->audiobuf)
    return -ENOMEM;
  return 0;
}

static int openRecorderInternal(struct recorder* rec, record_dev_id dev, WAVEFORMATEX* fmt, std::string base_path)
{
  int err = 0;
  err = snd_pcm_open((snd_pcm_t**)&rec->wavein_hdl, dev.u.name, SND_PCM_STREAM_CAPTURE, 0);
  if (err < 0)
    goto fail;

  err = set_params(rec, fmt, DEF_BUFF_TIME, DEF_PERIOD_TIME);
  if (err)
    goto fail;
  assert(rec->bufheader == NULL);

  err = prepareRecBuffer(rec);
  if (err)
    goto fail;

  err = createRecordThread((void*)rec, &rec->rec_thread, base_path);
  if (err)
    goto fail;

  return 0;

fail:
  if (rec->wavein_hdl)
    snd_pcm_close((snd_pcm_t*)rec->wavein_hdl);
  rec->wavein_hdl = NULL;

  free_rec_buffer(rec);

  return err;
}

static void closeRecorderInternal(struct recorder* rec)
{
  snd_pcm_t* handle;
  handle = (snd_pcm_t*)rec->wavein_hdl;
  /* may be the thread is blocked at read, cancel it */
  pthread_cancel(rec->rec_thread);

  /* wait for the pcm thread quit first */
  pthread_join(rec->rec_thread, NULL);
  if (handle)
  {
    snd_pcm_close(handle);
    rec->wavein_hdl = NULL;
  }

  free_rec_buffer(rec);
}

/* return the count of pcm device */

/* list all cards */

static int getPcmDeviceCount(snd_pcm_stream_t stream)
{
  void **hints, **n;
  char *io, *filter, *name;

  int cnt = 0;
  if (snd_device_name_hint(-1, "pcm", &hints) < 0)
    return 0;

  n = hints;
  filter = stream == SND_PCM_STREAM_CAPTURE ? (char*)"Input" : (char*)"Output";
  while (*n != NULL)
  {
    io = snd_device_name_get_hint(*n, "IOID");
    name = snd_device_name_get_hint(*n, "NAME");
    if (name && (io == NULL || strcmp(io, filter) == 0))
      cnt++;

    if (io != NULL)
      free(io);

    if (name != NULL)
      free(name);
    n++;
  }
  snd_device_name_free_hint(hints);
  return cnt;
}

/* -------------------------------------

 * Interfaces

 --------------------------------------*/

/* the device id is a pcm string name in linux */

record_dev_id getDefaultInputDevice()
{
  record_dev_id id;
  id.u.name = (char*)"default";
  return id;
}

record_dev_id* listInputDevice()
{
  // TODO: unimplemented
  return NULL;
}

int getInputDeviceNum()
{
  return getPcmDeviceCount(SND_PCM_STREAM_CAPTURE);
}

/* callback will be run on a new thread */
int createRecorder(struct recorder** out_rec, void (*on_data_ind)(char* data, unsigned long len, void* user_cb_para),
                   void* user_cb_para)
{
  struct recorder* myrec;
  myrec = (struct recorder*)malloc(sizeof(struct recorder));

  if (!myrec)
    return RECORD_ERROR_MALLOC_FAIL;

  memset(myrec, 0, sizeof(struct recorder));
  myrec->on_data_ind = on_data_ind;
  myrec->user_cb_para = user_cb_para;
  myrec->state = RECORD_STATE_CREATED;

  *out_rec = myrec;
  return RECORD_SUCCESS;
}

void destroyRecorder(struct recorder* rec)
{
  if (!rec)
    return;
  free(rec);
}

int openRecorder(struct recorder* rec, record_dev_id dev, WAVEFORMATEX* fmt, std::string base_path)
{
  std::cout << "rec->state is : " << rec->state << std::endl;
  int ret = 0;
  if (!rec)
    return -RECORD_ERROR_INVAL;
  if (rec->state >= RECORD_STATE_READY)
    return RECORD_SUCCESS;
  ret = openRecorderInternal(rec, dev, fmt, base_path);
  if (ret == 0)
    rec->state = RECORD_STATE_READY;
  return RECORD_SUCCESS;
}

void closeRecorder(struct recorder* rec)
{
  if (rec == NULL || rec->state < RECORD_STATE_READY)
    return;
  if (rec->state == RECORD_STATE_RECORDING)
    stopRecord(rec);
  rec->state = RECORD_STATE_CLOSING;
  closeRecorderInternal(rec);
  rec->state = RECORD_STATE_CREATED;
}

int startRecord(struct recorder* rec)
{
  int ret;
  if (rec == NULL)
    return -RECORD_ERROR_INVAL;
  if (rec->state < RECORD_STATE_READY)
    return -RECORD_ERROR_NOT_READY;

  if (rec->state == RECORD_STATE_RECORDING)
    return 0;
  ret = startRecordInternal((snd_pcm_t*)rec->wavein_hdl);

  if (ret == 0)
    rec->state = RECORD_STATE_RECORDING;
  return ret;
}

int stopRecord(struct recorder* rec)
{
  int ret;
  if (rec == NULL)
    return -RECORD_ERROR_INVAL;

  if (rec->state < RECORD_STATE_RECORDING)
    return 0;

  rec->state = RECORD_STATE_STOPPING;
  ret = stopRecordInternal((snd_pcm_t*)rec->wavein_hdl);

  if (ret == 0)
  {
    rec->state = RECORD_STATE_READY;
  }

  return ret;
}

int isRecordStopped(struct recorder* rec)
{
  if (rec->state == RECORD_STATE_RECORDING)
    return 0;
  return isStoppedInternal(rec);
}
