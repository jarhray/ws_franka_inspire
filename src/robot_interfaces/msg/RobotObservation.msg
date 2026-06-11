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

# Inspire tactile: normal/tangential forces per finger (same layout as GetTouchAct1.msg).
service_interfaces/GetTouchAct1 hand_touch

# Legacy single-camera fields: mirror the "primary" camera for backward compatibility.
# When multi-camera is used, aggregator fills these from default_camera_id (or first slot).
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/CameraInfo camera_info

# Multi-camera (parallel arrays: camera_ids[i] <-> rgb_images[i], depth_images[i], camera_infos[i]).
# Logical ids e.g. cam1, cam2 (configured in observation_aggregator).
string[] camera_ids
sensor_msgs/Image[] rgb_images
sensor_msgs/Image[] depth_images
sensor_msgs/CameraInfo[] camera_infos

string ee_pose_frame
