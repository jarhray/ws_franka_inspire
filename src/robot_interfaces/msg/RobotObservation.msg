std_msgs/Header header

# Arm (FR3/Franka) joint state, following a fixed joint order defined by the system.
float64[] arm_joint_position
float64[] arm_joint_velocity

# End-effector kinematics in `ee_pose_frame` (see frame convention in your pipeline).
geometry_msgs/Pose ee_pose
geometry_msgs/Twist ee_twist

# Hand (Inspire) joint state, following a fixed joint order defined by the system.
float64[] hand_joint_position
float64[] hand_joint_velocity

# Latest RealSense images for perception / policy.
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/CameraInfo camera_info

string ee_pose_frame

