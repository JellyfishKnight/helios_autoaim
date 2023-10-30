std_msgs/Header header

float32 yaw_angle             #云台的yaw
float32 pitch_angle           #云台的pitch
float32 bullet_speed          #子弹的射速
int8 target_mode              #发射模式(装甲板自瞄模式 或者 能量机关模式)
                              # 0 是自瞄 1 是小能量机关 2 是大能量机关 但是目前1，2没有区别
int8 target_color             #目标颜色 0为红色，1为蓝色