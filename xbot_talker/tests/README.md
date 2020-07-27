## 1. xbot_talker功能与使用简介

xbot_talker是用于重德智能XBot-U科研教学平台机器人的**语音交互**ROS程序包。

该程序包具有离线命令词识别、离线TTS以及离线对话等多种功能。

## 2. 安装依赖

安装命令行播放音乐工具：

```
sudo apt install sox
sudo apt install libsox-fmt-all
```

安装gflags：
```
sudo apt-get install libgflags-dev
```

官方开发文档：https://doc.xfyun.cn/msc_linux/

---

## 3. 使用方法 

在 `xbot_talker/` 中创建 `config/` 文件夹，将 `assets/` 中的 `grammar.bnf`, `new_dictionary.txt`, `gretting.txt` 复制到 `xbot_talker/config/` 中（refine soon）

在机器人上运行

```
roslaunch xbot_talker build_grammar.launch
```

```
roslaunch xbot_talker talker.launch
```

---
### 3.1 查看talker版本信息

`rosrun xbot_talker talker --version`或者直接执行可执行文件`./talker --version`可以看到输出的版本信息。

```
talker version XBOT Talker
Droid Intelligence Co.,Ltd.

 v2.0-dev
```



### 3.2 调用播放服务

#### 3.2.1 播放文字合成语音

在ROS系统上运行

```
rosservice call /xbot/play "loop: false
mode: 2
audio_path: ''
tts_text: '你好'" 
```
计算机（或者其它运行载体如机器人）会发出“你好”的声音提示。



#### 3.2.2 播放制定文件

在ROS系统上运行

```
rosservice call /xbot/play "loop: false
mode: 1
audio_path: '/home/xj/catkin_ws/src/xbot_talker/assets/wav/age.wav'
tts_text: ''" 
```
计算机（或者其它运行载体如机器人）会发出“我已经三岁了”声音。


### 3.3 调用对话服务

在ROS系统上运行：

```
rosservice call /xbot/chat "start_chat: true"
```

成功运行后，我们会听到“嘟”声。
当“嘟”声响起的时候，我们可以开始对话了，当前程序支持的对话内容在xbot_talker/assets/new_dictionary.txt 和 grammar.bnf中定义。
比如有：

| 你说的话                           | 机器人的回复                                                 |
| ---------------------------------- | ------------------------------------------------------------ |
| "你好"                             | 你好，我是机器人小德，请问有什么能帮您的？                   |
| "叫什么","名字"                    | "我叫小德，希望我能成为您的好伙伴，请多指教。"               |
| "你多大","几岁"                    | "我已经三岁啦。"                                             |
| "你来自"                           | "我来自重德智能公司"                                         |
| "介绍","你自己"                    | "您好，我是重德智能的机器人小德，集科研、教学、服务于一体的多功能机器人。" |
| "你是谁"                           | "我是小德呀，你不认识我吗？我可认识你哦！"                   |
| "再见"                             | "好的,期待下一次的和你相遇。"                                |
| "高兴","见到你"                    | "我和你的每一次遇见都是我的小确幸。"                         |
| "会做什么","你会什么","你会做什么" | 我可以和你一起学习,一起听歌，一起散步，做一切你想要我做的事情。" |
| "介绍","重德智能"                  | "北京中科重德智能科技有限公司是源自中国科学院的创业公司,公司致力于研发智能机器人科研教学平台，提供稳定可靠、安全高效的机器人软硬件一体化解决方案" |


针对xbot-u基础动作的命令识别与执行（机器人指令）：

| 命令 | 响应                   |
| ---- | ---------------------- |
| 关闭 | 机器人关闭语音交互功能 |

对机器人说“关闭”机器人将关闭对话服务。下次对话之前，需要使用命令```rosservice call /xbot/chat "start_chat: true"```再次启动对话服务后再次对话。

对话过程中，每次输入的关键词语音都会保存到audio文件夹下，以时间命名，保存的数据为pcm裸数据（不包含文件头信息），可复制，可用播放软件播放，也可作为测试数据输入`offlineVoiceRecogPCM`用于语音识别。


### 3.4 查看talker当前状态

```
rostopic echo /xbot/talk_state
```
命名窗口返回：
```
isChatting: True
isPlaying: False
play_mode: 0
---
isChatting: False
isPlaying: False
play_mode: 0
---
isChatting: True
isPlaying: False
play_mode: 0
---

```


### 4. 自定义对话内容

用户可以通过修改以下两个配置文件自定义对话内容。

1. 修改xbot_talker/assets/grammar.bnf语法文件。

bnf语法使用一种结构描述了用户可能说出的语言范围和构成模式。简单来说，机器人只能够识别该文件中定义的关键词。该文件可用WPS打开并编辑。打开文件后，用户可直接在的最后添加自定义的关键词，例如，添加关键词“你有弟弟吗”“你弟弟是谁”：   

```
<content>:向前走!id(9)|向前走一步!id(10)|向后走!id(11)|向后走一步!id(11)|向后退!id(12)|向后退一步!id(13)|向左转!id(14)|向左旋转!id(15)|向右转!id(16)|向右旋转!id(17)|你好!id(1001)|你叫什么名字!id(1002)|你多大了!id(1003)|你几岁了!id(1004)|你来自哪里!id(1005)|介绍一下你自己!id(1006)|你自我介绍一下!id(1007)|介绍你自己!id(1008)|再见!id(1009)|很高兴见到你!id(1010)|你会做什么!id(1011)|你会什么!id(1012)|你是谁!id(1013)|快递!id(1014)|介绍重德智能!id(1015)|介绍一下重德智能!id(1015)|你有弟弟吗!id(1016)|你弟弟是谁!id(1017)|抬起左手!id(1018)|抬起右手!id(1019)|左手拿!id(1020)|右手拿!id(1021)|放下左手!id(1022)|放下右手!id(1023)|张开左手!id(1024)|张开右手!id(1025)|左手抓住!id(1026)|右手抓住!id(1027)|好的!id(1028)|是的!id(1029)|小德你回去吧!id(1030);
```

​                                             

关键词之间用“| ”分隔开，“!id”后面小括号里的数字，依次往后加1即可，最后用分号“；”结束。

2. 修改xbot_talker/assets/new_dictionary.txt文件，添加新的交互词典。

打开new_dictionary.txt文件，在文件末尾，“}”和“]”之前添加自定义的交互信息。

```
        {
              "keyword":["你有弟弟吗","你弟弟是谁"],
              "answer":"我弟弟是漩涡鸣人，特别逗逼。",
                    "isAudio":1,
                                "generate_audio":"didi.wav",
              "flag":0
          },
```




每一组交互信息都用花括号“{}”括起来，每组信息之间用逗号“，”分隔开。

keyword必须是grammar.bnf定义的关键词，否则机器人无法识别。

answer代表自定义的回答，可以是回复的语句，也可以是控制机器人前进后退的指令。

isAudio标志，1表示把answer里的文字转化为语音播出，语音文件存放在自定义的generate_audio对应的【名称.wav文件】里。

flag标志，1表示进行语音回答后机器人不需要进行额外动作，0表示机器人可能需要进行移动等动作。

修改完以上两个配置文件之后，便可按照3.2 语音对话的方式对内容进行测试。

### 5. 语法文件生成工具使用说明？

<https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/157>

在机器人上运行以下命令生成语法文件：

```
rosrun xbot_talker build_grammar arg1 arg2
```

其中 `arg1` 为 xbot_talker 所在完整路径。比如 xbot_talker 路径为 `~/catkin_ws/src/xbot_talker` 则 `arg1` 即为该路径。

`arg2` 为指定的生成语法文件路径，该参数应该与 `/src/main.cpp` 文件中的 `GRAMMAR_BUILD_PATH` 相同。按照 `arg1` 的格式，默认指定为 `~/catkin_ws/src/xbot_talker/config`。

接着运行以下命令启动服务

```
roslaunch xbot_talker talker.launch
```

## 4. 测试列表

### 4.1 回归测试用例列表

| 测试用例编号 |
| ------------ |
| ts0          |
| ts1          |
| ts2          |
| ts3          |



### 4.2 MR之前需要开发自行测试的ts list

| 测试用例编号                              |
| ----------------------------------------- |
| ts0                                       |
| ts1                                       |
| ts2                                       |
| ts3                                       |
| ts4～ts9 任务对应的assignee负责各自的模块 |



### 4.3 MR到dev分支后测试角色测试的ts list

| 测试用例编号 |
| ------------ |
| ts0          |
| ts1          |
| ts2          |
| ts3          |
| ts10         |
| ts11         |
| ts12         |
| ts13         |
| ts14         |



### 4.4 准备内部发布的时候的 ts list



### 4.5 准备外部发布的时候的 ts list





## 5. 测试用例

#### ts0: 启动talker运行期间不崩溃
1. 启动talker.launch ```roslaunch xbot_talker talker.launch```成功，无错误。

#### ts1: 测试语音对话
1. 启动启动talker.launch ```roslaunch xbot_talker talker.launch```之后，说“你好”，程序无错误，能够得到语音反馈。

#### ts2: 测试播放文字合成语音

1. 运行
```
rosservice call /xbot/play "loop: false
mode: 2
audio_path: ''
tts_text: '你好'" 
```
程序无错误，会发出“你好”的声音提示。


#### ts3: 播放制定文件

1. 运行
```
rosservice call /xbot/play "loop: false
mode: 1
audio_path: '/home/xj/catkin_ws/src/xbot_talker/assets/wav/age.wav'
tts_text: ''" 
```
程序无错误，会发出“我已经三岁了”声音。



#### ts4: 测试用户配置文件剥离

##### ts4-1:测试用户手写配置文件分离   [#44](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/44)

**参数说明**

运行脚本文件即可实现对用户手写配置文件的移动。`sourcepath` 应该指向用户手写文件位置，`savepath` 指向希望存储的路径。

> **注意**：此处 `savepath` 的设置应该与相应 `build_grammar.launch` 与 `talker.launch` 文件中第二个参数设置相同。见 ts4-2
> 
> 此处通过复制对用户配置文件进行移动，目的是方便其它测试用例对于语法文件更改不会影响原始版本。且 git 时不会在仓库添加不必要更新。

**运行**

在 `catkin_ws/` 下运行命令

```
./src/xbot_talker/tests/ts4/ts4setConfig.sh
```

**结果**

在 `tests/ts4/` 中生成 `config` 文件夹，在文件夹内有用户放在 `sourcepath` 中的 `*.txt` 文件及 `*.bnf` 文件。

##### ts4-2:测试用户手写配置文件生成的语法文件分离   [#157](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/157)

**参数说明**

运行 `ts4_build_grammar.launch` 文件即可实现对用户手写配置文件的语法文件生成。

- `arg[1]`
  讯飞 sdk 相关文件存储路径
- `arg[2]`
  用户手写配置存储路径，*此处应与 ts4-1 中 `savepath` 参数相同*

运行 `ts4_talker.launch` 可以正常运行 talker

- `param 'base_path'`
  讯飞 sdk 相关文件存储路径
- `param 'grammar_file'`
  用户手写配置文件生成的语法文件存储路径，*此处应与 ts4-1 中 `savepath` 参数相同*

**运行**

在 `catkin_ws/` 下运行命令

```
roslaunch xbot_talker ts4_build_grammar.launch
roslaunch xbot_talker ts4_talker.launch
```

**结果**

运行 `roslaunch xbot_talker ts4_build_grammar.launch` 命令后于 `tests/ts4/config/` 文件夹内生成 `grammar` 文件夹，里面
有用户手写配置生成的语法文件夹 `temp/` 以及记录语法信息的 `info.txt` 文件，*相关参数内容可参见 `doc/syntax_of_asr_config.md`*

运行 `roslaunch xbot_talker ts4_talker.launch` 命令后会启动 talker，对话测试与 ts1 没有区别。

#### ts5: 测试asr节点分离   [#194](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/194)
##### ts5-1:测试读取pcm文件用于音频识别。
1.构建语法同ts1无任何区别：

- roslaunch xbot_talker ts5_build_grammar.launch 

2.测试write_audio_and_recog.cpp：

- 启动`roslaunch xbot_talker ts5_write_audio.launch `可看到终端输出完整的语音识别结果（json）

 在tests/ts5/testdemo/write_audio_and_recog.cpp内修改

```
    // FIXME:pcm测试文件的选取后续改用gflags实现
    std::string pcmFile = "/nihao_test.pcm";

```

代码中pcmFile的值可测试不同的文件。

##### ts5-2:测试一次录音并保存录音文件
- 启动`roslaunch xbot_talker ts5_recod_once.launch  `可看到终端输出`[ INFO] [1567001690.256014554]: -----------Start Recording--------`的提示，开始说话录音（约3s）.

- 当看到终端提示`[ INFO] [1567001693.514242419]: -----------Finish Recording--------`时录音结束，可到testaudio目录下查看保存的录音文件，此录音文件可用于ts5-1的测试，也可通过软件播放。

##### ts5-3:测试语音识别节点
- 先启动asr语音识别节点：`roslaunch xbot_talker ts5_voice_recog.launch `

- 新开一个终端，输入`rostopic echo /xbot_talker/final_result `，在新开一个终端，输入`rostopic echo /xbot_talker/recog_result`用于测试后面发布result消息的时候是否发布成功。假如出现`WARNING: topic [/xbot_talker/recog_result] does not appear to be published yet`警告，不用管，这是因为程序还没有进行到注册topic那一步。

- 再新开一个终端，输入`rosservice call /xbot/start_recog "start_recog: true" `通过服务请求启动语音识别（PS：这里的测试搞得这么复杂主要是为了后面和main.cpp对接。实际上可以修改代码直接启动节点进行语音识别）。

- 此时听到“嘟”声，在launch的终端界面也可看到开始录音的相关提示，嘟声后说关键词，可看到识别结果，并在topic检测的终端看到识别结果。

- recog_result表示识别的原始json串，final_result表示的json串处理后得到的识别字符和置信度。


- `尚且存在的问题：1.不能循环录音；2.即便final_result是一直发布，受总线程时间影响（约4s）,发送的帧数很有限，同时也出现了两个topic发布互相影响的问题（类似app测试时遇到的通信问题）`


##### ts5-4:测试main_test.cpp

- 先启动asr语音识别节点：`roslaunch xbot_talker ts5_voice_recog.launch `

- 启动仿照main.cpp的main_test节点,` roslaunch xbot_talker ts5_talker.launch `

- 新开终端，启动对话服务：`rosservice call /xbot/chat "start_chat: true"`

- 可听到嘟声并在asrlaunch终端看到提示，说关键词后会显示识别结果，同样可开启话题监听，看是否有发布的识别结果消息。

- `运气好的话ORZ`，可在ts5_talker.launch终端下看到输出的`Final result:你好|confidence:95`信息，说明main_test节点顺利接收到了asr发布的final_result信息。

- `尚且存在的问题：1.同上，不能循环录音；2.通信有问题，asr发布九条信息，main_test也就能收到2条，掉帧严重，这个跟app和ROS通信遇到的问题一模一样。这个问题我还是头一次在纯ROS的通信中遇到。`

#### ts6: 测试 tts 节点分离   [#214](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/214)


##### ts6-1:测试一次文本转换音频并保存音频文件

**参数说明**

目前节点运行与主程序没有关联。只会涉及 `text_to_speech_server.cpp` `request_text_to_speech_client.cpp` 以及 `ts6_tts.launch`。程序参数由 `ts6_tts.launch` 文件控制，可以根据需要进行调整。

- `arg[1]`
  讯飞 sdk 相关文件保存位置
- `arg[2]`
  音频文件保存路径
- `arg[3]`
  音频文件保存名称
- `arg[4]`
  要转换的文本信息

**运行**

运行命令

```
roslaunch xbot_talker ts6_tts.launch
```

**结果**

在 `tests/ts6/` 中生成文件夹，并在文件夹内生成相应的语音文件。



#### ts7: MIC读取音频后以文件传递给ASR识别   [#119](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/119)





#### ts8: 预留给[#46](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/46)





#### ts9: 预留给[#45](https://yt.droid.ac.cn/xbot-u/xbot_talker/issues/45)





#### ts10: 测试自定义对话内容

1. 按照“自定义对话内容”章节的使用说明，在xbot_talker/assets/grammar.bnf文件中新增一个关键词（自己定义），并在xbot_talker/assets/new_dictionary.txt中新增一个对话。

2. 按照ts1操作启动对话服务后，对新增的对话进行语音对话测试。要求你说的话包含定义的关键词，计算机输出的语音内容与对话库中定义的回答一致。

   

#### ts11: 启动talker连续对话半小时不崩溃

启动talker.launch ```roslaunch xbot_talker talker.launch```之后连续对话不少于半小时程序不崩溃。

- 不崩溃是指：程序能一直运行。不卡死，不异常退出，哪怕程序这次没有按照预期正常返回结果，但是在不重启的talker的情况下，依然能够保证下一次对话（播放）正常运行。
- 操作步骤参考ts0-ts1



#### ts12: 测试已定义的对话关键词能得到预期回应

在语音对话模式下，按照对话库定义的“命令词”和“应答内容”，测试是否能够给出预期的回复。



#### ts13: 测试未定义的对话关键词程序能给出信息提示

对语法文件未定义的关键词进行对话测试，程序能够正常应对，给出信息提示，无错误，不崩溃和退出。

#### 

#### ts14: 连续对话不少于25次，语音拾取提示嘟声间隔正常

按照语音对话使用方法，根据问答库，连续对话不少于25次，语音拾取提示嘟声间隔正常，能够保持正常对话状态。
