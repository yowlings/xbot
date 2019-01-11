# Xbot Core Sensor Data Messages

###### MESSAGE ######

Header header

###################
# Timestamp and battery voltage
###################
uint16 time_stamp      # milliseconds starting when turning on Xbot (max. 65536, then starts from 0 again)
uint16 left_encoder
uint16 right_encoder
bool ischarging
uint8 battery_percent

uint16 left_echo
uint16 center_echo
uint16 right_echo

uint16 front_infrared
uint16 rear_infrared

bool motor_disabled

float32 left_motor_current
float32 right_motor_current

uint16 error_state



