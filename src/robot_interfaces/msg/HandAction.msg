std_msgs/Header header

# Hand control mode:
#   0: joint_position
uint8 control_mode

bool is_relative

float64[] joint_position
float64[] joint_velocity

float64 duration_sec

