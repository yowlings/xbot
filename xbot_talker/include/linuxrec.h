/*
 * a simple record code. using alsa-lib APIs.
 * keep the function same as winrec.h
 *
 * Common steps:
 *	create_recorder,
 *	open_recorder,
 *	start_record,
 *	stop_record,
 *	close_recorder,
 *	destroy_recorder
 *
 */

#ifndef __IFLY_WINREC_H__
#define __IFLY_WINREC_H__
#include <alsa/asoundlib.h>
#include <pthread.h>
#include <iostream>
#include "file_operation.h"
#include "formats.h"
enum
{
  RECORD_STATE_CREATED, /* Init		*/
  RECORD_STATE_CLOSING,
  RECORD_STATE_READY,     /* Opened	*/
  RECORD_STATE_STOPPING,  /* During Stop	*/
  RECORD_STATE_RECORDING, /* Started	*/
};

typedef struct
{
  union
  {
    char* name;
    int index;
    void* resv;
  } u;
} record_dev_id;

/* recorder object. */
struct recorder
{
  void (*on_data_ind)(char* data, unsigned long len, void* user_para);
  void* user_cb_para;
  volatile int state; /* internal record state */
  void* wavein_hdl;
  /* thread id may be a struct. by implementation
   * void * will not be ported!! */
  pthread_t rec_thread;
  /*void * rec_thread_hdl;*/
  void* bufheader;
  unsigned int bufcount;
  char* audiobuf;
  int bits_per_frame;
  unsigned int buffer_time;
  unsigned int period_time;
  size_t period_frames;
  size_t buffer_frames;
  std::string pcm_file_path;
};

#if __cplusplus
extern "C" {
#endif /* C++ */

class RecordAlsaAPI
{
public:
  void initRecord(struct recorder* rec, record_dev_id dev, WAVEFORMATEX* fmt);
  int setRecordDuration(const float duration_time);
  struct DataBuff startRecord();
  void closeRecord();

private:
  struct recorder* record_;
  snd_pcm_hw_params_t* params_;  //指向 snd_pcm_hw_params_t的指针
  unsigned int record_rate_;     //录音时候的采样率
  snd_pcm_format_t format_;
  snd_pcm_uframes_t frames_;
  snd_pcm_t* handle_;
  int audiobuf_size_;
  int duration_time_;
  struct DataBuff audio_pcm_;
  int record_loop_;
};

class VDADetection
{
public:
  float levelEnergy(struct DataBuff pcm_data, const int call_count);
  float energyPerSample(struct DataBuff pcm_data);

private:
  int call_count_ = 0;
  float background_energy_ = 0.0;
  float temp_energy_ = 0.0;
  float mean_energy_ = 0.0;
  float level_ = 0;
  const float forgetfactor_ = 1;
  const float adjustment_ = 0.05;
  int is_speech_count_;
};
/**
 * @fn
 * @brief	Get the default input device ID
 * @return	returns "default" in linux.
 */
record_dev_id getDefaultInputDevice();

/**
 * @fn
 * @brief	Get the total number of active input devices.
 * @return
 */
int getInputDeviceNum();

/**
 * @fn
 * @brief	Create a recorder object.
 *
 * Never call the close_recorder in the callback function. as close
 * action will wait for the callback thread to quit.
 *
 * @return	int	    - Return 0 in success, otherwise return error code.
 * @param	out_rec		- [out] recorder object holder
 * @param	on_data_ind	- [in]	callback. called when data coming.
 * @param	user_cb_para	- [in] user params for the callback.
 * @see
 */
int createRecorder(struct recorder** out_rec,

                   void (*on_data_ind)(char* data, unsigned long len, void* user_para),

                   void* user_cb_para);

/**
 * @fn
 * @brief	Destroy recorder object. free memory.
 * @param	rec	- [in]recorder object
 */
void destroyRecorder(struct recorder* rec);

/**
 * @fn
 * @brief	open the device.
 * @return	int	                - Return 0 in success, otherwise return
 * error code.
 * @param	rec			- [in] recorder object
 * @param	dev			- [in] device id, from 0.
 * @param	fmt			- [in] record format.
 * @see
 * 	get_default_input_dev()
 */
int openRecorder(struct recorder* rec, record_dev_id dev, WAVEFORMATEX* fmt, std::string base_path);

/**
 * @fn
 * @brief	close the device.
 * @param	rec			- [in] recorder object
 */
void closeRecorder(struct recorder* rec);

/**
 * @fn
 * @brief	start record.
 * @return	int			- Return 0 in success, otherwise return
 * error
 * code.
 * @param	rec			- [in] recorder object
 */
int startRecord(struct recorder* rec);

/**
 * @fn
 * @brief	stop record.
 * @return	int			- Return 0 in success, otherwise return
 * error
 * code.
 * @param	rec			- [in] recorder object
 */
int stopRecord(struct recorder* rec);

/**
 * @fn
 * @brief	test if the recording has been stopped.
 * @return	int			- 1: stopped. 0 : recording.
 * @param	rec			- [in] recorder object
 */
int isRecordStopped(struct recorder* rec);

#if __cplusplus
} /* extern "C" */

#endif /* C++ */

#endif
