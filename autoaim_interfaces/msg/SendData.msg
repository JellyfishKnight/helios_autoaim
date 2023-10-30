std_msgs/Header header

float32 yaw         #预测得出的装甲板的位置相对于相机的yaw
float32 pitch       #预测的出的装甲板的位置相对于相机的pitch
int8 number            #识别到的装甲板的数字
int8 find           #视野内是否有目标 0 为否 1 为是
int8 cmd            #1 为发射