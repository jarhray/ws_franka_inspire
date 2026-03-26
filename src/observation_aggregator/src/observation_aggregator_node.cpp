#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robot_interfaces/msg/robot_observation.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "service_interfaces/msg/get_angle_act1.hpp"

class ObservationAggregator : public rclcpp::Node
{
public:
  ObservationAggregator()
  : rclcpp::Node("observation_aggregator")
  {
    using std::placeholders::_1;

    // Topics
    arm_joint_state_topic_ = declare_parameter<std::string>(
      "arm_joint_state_topic", "~/measured_joint_states");
    arm_pose_topic_ = declare_parameter<std::string>("arm_pose_topic", "~/current_pose");
    arm_twist_topic_ = declare_parameter<std::string>(
      "arm_twist_topic", "~/desired_end_effector_twist");

    hand_angle_topic_ = declare_parameter<std::string>("hand_angle_topic", "/angle_data");

    rs_rgb_image_topic_ =
      declare_parameter<std::string>("rs_rgb_image_topic", "/camera/color/image_raw");
    rs_depth_image_topic_ =
      declare_parameter<std::string>("rs_depth_image_topic", "/camera/depth/image_rect_raw");
    rs_camera_info_topic_ =
      declare_parameter<std::string>("rs_camera_info_topic", "/camera/color/camera_info");

    observation_topic_ = declare_parameter<std::string>("observation_topic", "/robot/observation");

    // Joint/Finger order mapping (fixed order required by RobotObservation.msg)
    hand_finger_id_order_ = declare_parameter<std::vector<int64_t>>(
      "hand_finger_id_order", std::vector<int64_t>{0, 1, 2, 3, 4, 5});
    arm_joint_name_order_ =
      declare_parameter<std::vector<std::string>>("arm_joint_name_order", std::vector<std::string>{});

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);

    // Subscribers
    auto sensor_qos = rclcpp::SensorDataQoS();
    arm_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_joint_state_topic_, sensor_qos, std::bind(&ObservationAggregator::onArmJoint, this, _1));
    arm_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      arm_pose_topic_, sensor_qos, std::bind(&ObservationAggregator::onArmPose, this, _1));
    arm_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      arm_twist_topic_, sensor_qos, std::bind(&ObservationAggregator::onArmTwist, this, _1));
    hand_angle_sub_ = create_subscription<service_interfaces::msg::GetAngleAct1>(
      hand_angle_topic_, sensor_qos, std::bind(&ObservationAggregator::onHandAngle, this, _1));

    rs_rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rs_rgb_image_topic_, sensor_qos, std::bind(&ObservationAggregator::onRgbImage, this, _1));
    rs_depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rs_depth_image_topic_, sensor_qos,
      std::bind(&ObservationAggregator::onDepthImage, this, _1));
    rs_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      rs_camera_info_topic_, sensor_qos,
      std::bind(&ObservationAggregator::onCameraInfo, this, _1));

    // Publisher
    observation_pub_ = create_publisher<robot_interfaces::msg::RobotObservation>(
      observation_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_)),
      std::bind(&ObservationAggregator::onTimer, this));
  }

private:
  static bool isFinite(double v)
  {
    return std::isfinite(v);
  }

  static builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time& t)
  {
    // Manual conversion to support Humble API differences.
    const int64_t ns = t.nanoseconds();
    builtin_interfaces::msg::Time out;
    int64_t sec64 = ns / 1000000000LL;
    int64_t rem64 = ns % 1000000000LL;  // may be negative
    if (rem64 < 0) {
      // Handle negative timestamps (rare for rosbag playback).
      rem64 += 1000000000LL;
      sec64 -= 1;
    }
    out.sec = static_cast<int32_t>(sec64);
    out.nanosec = static_cast<uint32_t>(rem64);
    return out;
  }

  void onArmJoint(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_arm_joint_state_ = *msg;
    has_arm_joint_state_ = true;
    arm_stamp_ = rclcpp::Time(msg->header.stamp);
    has_arm_stamp_ = true;
  }

  void onArmPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_arm_pose_ = *msg;
    has_arm_pose_ = true;
    arm_stamp_ = rclcpp::Time(msg->header.stamp);
    has_arm_stamp_ = true;
  }

  void onArmTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_arm_twist_ = *msg;
    has_arm_twist_ = true;
    arm_stamp_ = rclcpp::Time(msg->header.stamp);
    has_arm_stamp_ = true;
  }

  void onHandAngle(const service_interfaces::msg::GetAngleAct1::SharedPtr msg)
  {
    const auto now = this->now();

    // Build a lookup table: finger_id -> angle
    std::unordered_map<int32_t, double> angle_by_id;
    angle_by_id.reserve(msg->finger_ids.size());
    const size_t n = std::min(msg->finger_ids.size(), msg->angles.size());
    for (size_t i = 0; i < n; ++i) {
      angle_by_id[msg->finger_ids[i]] = static_cast<double>(msg->angles[i]);
    }

    std::vector<double> pos(hand_finger_id_order_.size(),
      std::numeric_limits<double>::quiet_NaN());
    for (size_t j = 0; j < hand_finger_id_order_.size(); ++j) {
      const int32_t fid = static_cast<int32_t>(hand_finger_id_order_[j]);
      auto it = angle_by_id.find(fid);
      if (it != angle_by_id.end()) {
        pos[j] = it->second;
      }
    }

    std::vector<double> vel(hand_finger_id_order_.size(), 0.0);
    if (has_last_hand_ && hand_finger_id_order_.size() == last_hand_position_.size()) {
      const double dt = (now - last_hand_time_).seconds();
      if (dt > 1e-6) {
        for (size_t j = 0; j < pos.size(); ++j) {
          if (isFinite(pos[j]) && isFinite(last_hand_position_[j])) {
            vel[j] = (pos[j] - last_hand_position_[j]) / dt;
          }
        }
      }
    }

    last_hand_position_ = std::move(pos);
    last_hand_velocity_ = std::move(vel);
    has_last_hand_ = true;
    last_hand_time_ = now;
    has_hand_stamp_ = true;
    hand_stamp_ = now;
  }

  void onRgbImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_rgb_image_ = *msg;
    has_rgb_image_ = true;
    rs_stamp_ = rclcpp::Time(msg->header.stamp);
    has_rs_stamp_ = true;
  }

  void onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_depth_image_ = *msg;
    has_depth_image_ = true;
    rs_stamp_ = rclcpp::Time(msg->header.stamp);
    has_rs_stamp_ = true;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    last_camera_info_ = *msg;
    has_camera_info_ = true;
    rs_stamp_ = rclcpp::Time(msg->header.stamp);
    has_rs_stamp_ = true;
  }

  std::vector<double> buildArmPositions() const
  {
    if (!has_arm_joint_state_) {
      return {};
    }

    if (arm_joint_name_order_.empty()) {
      if (last_arm_joint_state_.position.empty()) {
        return {};
      }
      return last_arm_joint_state_.position;
    }

    std::unordered_map<std::string, double> pos_by_name;
    pos_by_name.reserve(last_arm_joint_state_.name.size());
    for (size_t i = 0; i < last_arm_joint_state_.name.size(); ++i) {
      if (i < last_arm_joint_state_.position.size()) {
        pos_by_name[last_arm_joint_state_.name[i]] = last_arm_joint_state_.position[i];
      }
    }

    std::vector<double> out;
    out.reserve(arm_joint_name_order_.size());
    for (const auto& name : arm_joint_name_order_) {
      auto it = pos_by_name.find(name);
      out.push_back(it == pos_by_name.end() ? 0.0 : it->second);
    }
    return out;
  }

  std::vector<double> buildArmVelocities() const
  {
    if (!has_arm_joint_state_) {
      return {};
    }

    const bool has_velocity = !last_arm_joint_state_.velocity.empty();

    if (arm_joint_name_order_.empty()) {
      if (!has_velocity) {
        // Keep array sizes consistent with positions.
        return std::vector<double>(last_arm_joint_state_.position.size(), 0.0);
      }
      return last_arm_joint_state_.velocity;
    }

    std::unordered_map<std::string, double> vel_by_name;
    vel_by_name.reserve(last_arm_joint_state_.name.size());
    for (size_t i = 0; i < last_arm_joint_state_.name.size(); ++i) {
      if (i < last_arm_joint_state_.velocity.size()) {
        vel_by_name[last_arm_joint_state_.name[i]] = last_arm_joint_state_.velocity[i];
      }
    }

    std::vector<double> out;
    out.reserve(arm_joint_name_order_.size());
    for (const auto& name : arm_joint_name_order_) {
      auto it = vel_by_name.find(name);
      out.push_back(it == vel_by_name.end() ? 0.0 : it->second);
    }
    return out;
  }

  void onTimer()
  {
    robot_interfaces::msg::RobotObservation obs;
    const auto now = this->now();

    // For now, publish a "best available" timestamp (will be refined later for sync).
    rclcpp::Time stamp = now;
    if (has_arm_stamp_) {
      stamp = std::max(stamp, arm_stamp_);
    }
    if (has_hand_stamp_) {
      stamp = std::max(stamp, hand_stamp_);
    }
    if (has_rs_stamp_) {
      stamp = std::max(stamp, rs_stamp_);
    }
    obs.header.stamp = toBuiltinTime(stamp);

    // Arm
    obs.arm_joint_position = buildArmPositions();
    obs.arm_joint_velocity = buildArmVelocities();

    if (has_arm_pose_) {
      obs.ee_pose = last_arm_pose_.pose;
      obs.ee_pose_frame = last_arm_pose_.header.frame_id;
    }
    if (has_arm_twist_) {
      obs.ee_twist = last_arm_twist_.twist;
    }

    // Hand (computed velocity from successive GetAngleAct1 callbacks)
    if (has_last_hand_) {
      obs.hand_joint_position = last_hand_position_;
      obs.hand_joint_velocity = last_hand_velocity_;
    }

    // RealSense
    if (has_rgb_image_) {
      obs.rgb_image = last_rgb_image_;
    }
    if (has_depth_image_) {
      obs.depth_image = last_depth_image_;
    }
    if (has_camera_info_) {
      obs.camera_info = last_camera_info_;
    }

    observation_pub_->publish(obs);
  }

private:
  // Topics
  std::string arm_joint_state_topic_;
  std::string arm_pose_topic_;
  std::string arm_twist_topic_;
  std::string hand_angle_topic_;
  std::string rs_rgb_image_topic_;
  std::string rs_depth_image_topic_;
  std::string rs_camera_info_topic_;
  std::string observation_topic_;

  double publish_rate_hz_{30.0};

  // Parameters: fixed joint order
  std::vector<int64_t> hand_finger_id_order_;
  std::vector<std::string> arm_joint_name_order_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_sub_;
  rclcpp::Subscription<service_interfaces::msg::GetAngleAct1>::SharedPtr hand_angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs_rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rs_camera_info_sub_;

  // Publisher
  rclcpp::Publisher<robot_interfaces::msg::RobotObservation>::SharedPtr observation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Cached latest data + stamps
  sensor_msgs::msg::JointState last_arm_joint_state_;
  geometry_msgs::msg::PoseStamped last_arm_pose_;
  geometry_msgs::msg::TwistStamped last_arm_twist_;
  bool has_arm_joint_state_{false};
  bool has_arm_pose_{false};
  bool has_arm_twist_{false};

  rclcpp::Time arm_stamp_;
  bool has_arm_stamp_{false};

  std::vector<double> last_hand_position_;
  std::vector<double> last_hand_velocity_;
  bool has_last_hand_{false};
  rclcpp::Time last_hand_time_;
  bool has_hand_stamp_{false};
  rclcpp::Time hand_stamp_;

  sensor_msgs::msg::Image last_rgb_image_;
  sensor_msgs::msg::Image last_depth_image_;
  sensor_msgs::msg::CameraInfo last_camera_info_;
  bool has_rgb_image_{false};
  bool has_depth_image_{false};
  bool has_camera_info_{false};

  rclcpp::Time rs_stamp_;
  bool has_rs_stamp_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObservationAggregator>());
  rclcpp::shutdown();
  return 0;
}

