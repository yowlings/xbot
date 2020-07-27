# xbot(metapackage) 

The file introduces every package subscribe topics,publish topic,services.

## xbot_node

### Publish

1. /mobile_base/joint_states (<sensor_msgs::JointState>)
2. /mobile_base/sensors/core(<xbot_msgs::CoreSensor>)
3. /mobile_base/sensors/extra(<xbot_msgs::ExtraSensor>)
4. /mobile_base/sensors/yaw_platform_degree(<std_msgs::Int8>)
5. /mobile_base/sensors/pitch_platform_degree(<std_msgs::Int8>)
6. /mobile_base/sensors/motor_enabled(<std_msgs::Bool>)
7. /mobile_base/sensors/sound_enabled(<std_msgs::Bool>)
8. /mobile_base/snesors/battery(<xbot_msgs::Battery>)
9. /mobile_base/sensors/front_echo(<sensor_msgs::Range>)
10. /mobile_base/sensors/rear_echo(<sensor_msgs::Range>)
11. /mobile_base/sensors/infrared(<xbot_msgs::InfraRed>)
12. /imu(<sensor_msgs::Imu>)
13. /odom(<nav_msgs::Odometry>)
14. /tf(<geometry_msgs::TransformStamped>)
15. /mobile_base/sensors/raw_imu_data(<xbot_msgs::RawImu>)
16. /mobile_base/xbot/state(<xbot_msgs::XbotState>)

### Subscribe

1. /mobile_base/commands/motor_enable(<std_msgs::Bool>)
2. /mobile_base/commands/velocity(<geometry_msgs::Twist>)
3. /mobile_base/commands/yaw_platform(<std_msgs::Int8>)[-90~90]
4. /mobile_base/commands/pitch_platform(<std_msgs::Int8>)[-60~30]
5. /mobile_base/commands/sound_enable(<std_msgs::Bool>)
6. /mobile_base/commands/led(<std_msgs::UInt8>)
7. /mobile_base/commands/reset_odometry(<std_msgs::Empty>)

## xbot_face

### Publish

1. /xbot/face_result(<xbot_face::FaceResult>)
2. /xbot/camera/image(<sensor_msgs::Image>)

### Subscribe

1. /xbot/camera/image(<sensor_msgs::Image>)



## xbot_talker

### Publish

1. /cmd_vel_mux/input/teleop(<geometry_msgs::Twist>)
2. /xbot/talker_state(<xbot_talker::talk_monitor>)
3. /welcome/leave(<std_msgs::Bool>)

### Subscribe

None

### ServiceServer

1. /xbot/play(<xbot_talker::play>)
2. /xbot/chat(<xbot_talker::chat>)