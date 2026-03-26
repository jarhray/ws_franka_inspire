std_msgs/Header header

# Control mode:
#   0: joint_position
#   1: joint_velocity
#   2: cartesian_pose
#   3: cartesian_velocity
uint8 control_mode

# If true, targets are interpreted as relative deltas to the current state.
bool is_relative

# Frame used for cartesian targets; ignored for joint_* modes.
string reference_frame

float64[] joint_position
float64[] joint_velocity

geometry_msgs/Pose cartesian_pose
geometry_msgs/Twist cartesian_velocity

# Desired execution duration (seconds). Executors may clamp.
float64 duration_sec

