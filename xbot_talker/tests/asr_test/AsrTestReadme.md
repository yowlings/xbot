### 01.通过mic录音进行离线或者在线命令词识别

通过修改mic_recog.launch里的 enable_online和 enable_offline参数的值，可以选择用科大讯飞离线命令词识别或者百度在线语音识别。

- 启动asr模块:

```
roslaunch xbot_talker mic_recog.launch 
```

- 通过发布topic控制开启一次录音识别:

```
rostopic pub -1 /xbot_talker/awaken_status xbot_talker/awaken_status "is_awaken: true
enable_chat: false" `
```


### 02.读取一个单声道pcm文件用于离线或者在线命令词识别

通过修改one_channel_recog.launch里的 enable_online和 enable_offline参数的值，可以选择用科大讯飞离线命令词识别或者百度在线语音识别。

- 修改one_channel_recog.launch里的`pcm_file`参数值为用于测试的单声道pcm音频文件。

- 启动asr模块:

```
roslaunch xbot_talker one_channel_recog.launch 

```

- 通过发布topic控制开启一次录音识别得到识别结果:

```
rostopic pub -1 /xbot_talker/awaken_status xbot_talker/awaken_status "is_awaken: true
enable_chat: false" 
```

### 03.读取一个双声道pcm文件用于离线命令词识别

- 修改two_channel_recog.launch.launch里的`pcm_file`参数值为用于测试的双声道pcm音频文件。

- 启动asr模块:

```
roslaunch xbot_talker two_channel_recog.launch 

```

- 通过发布topic控制开启一次录音识别得到识别结果:

```
rostopic pub -1 /xbot_talker/awaken_status xbot_talker/awaken_status "is_awaken: true
enable_chat: false" 

```
### 04.挨个读入单声道 PCM Data 文件用于语音识别测试并输出识别结果
修改usefiles-asr.launch文件里的enable_online和 enable_offline参数的值，可选择通过在线识别还是离线识别。
修改test_dir_path可以选择用于测试的音频文件夹。
修改log_path可以选择日志txt的保存位置。

测试时启动`roslaunch xbot_talker usefiles-asr.launch`即可。
