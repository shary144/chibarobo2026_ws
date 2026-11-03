std_msgs/Header header
uint8 system_state          # 0:EMERGENCY, 1:READY, 2:DRIVE
uint16 pid_configured_mask  # 設定完了フラグ
float32[16] angle
float32[16] velocity
int16[16] current
uint8[16] temp