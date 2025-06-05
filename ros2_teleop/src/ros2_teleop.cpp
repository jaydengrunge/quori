/*
    Jayden Grunde

    Node for taking controller inputs and publishing commands to Quori's joints and base.

*/ 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <unordered_map>
#include <string>
#include <optional>
#include <cstdint>
#include <iostream>

enum class Joint : std::uint8_t {
  LeftArm1,
  LeftArm2,
  RightArm1,
  RightArm2,
  WaistHinge,
  BaseLeft,
  BaseRight,
  BaseTurret
};

namespace std {
  template<>
  struct hash<Joint> {
    size_t operator()(const Joint &joint) const noexcept {
      return hash<std::uint8_t>()(static_cast<std::uint8_t>(joint));
    }
  };
}

const static std::unordered_map<Joint, std::string> JOINT_NAMES{
  {Joint::BaseLeft, "l_wheel"},
  {Joint::BaseRight, "r_wheel"},
  {Joint::BaseTurret, "turret"},
  {Joint::LeftArm1, "l_shoulder_pitch"},
  {Joint::LeftArm2, "l_shoulder_roll"},
  {Joint::RightArm1, "r_shoulder_pitch"},
  {Joint::RightArm2, "r_shoulder_roll"},
  {Joint::WaistHinge, "waist_pitch"}
};

struct XboxControllerState {
  bool dpad[4]{};
  double left_stick[2]{};
  double right_stick[2]{};
  bool left_trigger_1{};
  bool right_trigger_1{};
  double left_trigger_2{};
  double right_trigger_2{};
  bool a_button{};

  static XboxControllerState fromJoy(const sensor_msgs::msg::Joy::SharedPtr &msg) {
    XboxControllerState state;
    state.left_stick[0] = msg->axes[0];
    state.left_stick[1] = msg->axes[1];
    state.left_trigger_2 = msg->axes[2];
    state.right_stick[0] = msg->axes[3];
    state.right_stick[1] = msg->axes[4];
    state.right_trigger_2 = msg->axes[5];

    state.a_button = msg->buttons[0];
    state.left_trigger_1 = msg->buttons[4];
    state.right_trigger_1 = msg->buttons[5];

    return state;
  }
};

class QuoriTeleop : public rclcpp::Node {
public:
  QuoriTeleop()
  : Node("quori_teleop")
  {
    using std::placeholders::_1;

    traj_.joint_names = {
      "r_shoulder_pitch",
      "r_shoulder_roll",
      "l_shoulder_pitch",
      "l_shoulder_roll",
      "waist_pitch"
    };

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&QuoriTeleop::onJoy, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&QuoriTeleop::onJointStates, this, _1));
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/quori/joint_trajectory_controller/command", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/quori/base_controller/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(40),
      std::bind(&QuoriTeleop::publishCommands, this));

    // Initialize specific joint states
    auto dummy_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    dummy_joint_state_msg->name = {
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "waist_pitch"
    };
    dummy_joint_state_msg->position = {
    0.0,  // r_shoulder_pitch
    -2.9,  // r_shoulder_roll
    0.0, // l_shoulder_pitch
    -1.2,  // l_shoulder_roll
    0.0   // waist_pitch
    };
    latest_joint_state_ = dummy_joint_state_msg;


    // Call onJoy once at startup with a dummy Joy message
    auto dummy_joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
    dummy_joy_msg->axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    dummy_joy_msg->buttons = {0, 0, 0, 0, 0, 0};
    onJoy(dummy_joy_msg);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  geometry_msgs::msg::Twist vel_;
  trajectory_msgs::msg::JointTrajectory traj_;

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = msg;
  }

  double lookupLatestPosition(const std::string &name) {
    if (!latest_joint_state_) return 0.0;

    for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
      if (latest_joint_state_->name[i] == name) {
        std::cout << name << ": " << latest_joint_state_->position[i] << std::endl;
        return latest_joint_state_->position[i];
      }
    }
    return 0.0;
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    vel_ = geometry_msgs::msg::Twist();
    traj_.points.clear();

    XboxControllerState state = XboxControllerState::fromJoy(msg);

    std::optional<double> left_arm_r1, left_arm_r2;
    std::optional<double> right_arm_r1, right_arm_r2;
    std::optional<double> waist_hinge;

    if (state.left_trigger_2 < -0.5) {
      left_arm_r1 = state.left_stick[1] * 2 * (state.a_button ? 2 : 1);
      left_arm_r2 = state.left_stick[0] * 2 * (state.a_button ? 2 : 1);
    } else if (state.right_trigger_2 < -0.5) {
      right_arm_r1 = state.left_stick[1] * 2 * (state.a_button ? 2 : 1);
      right_arm_r2 = state.left_stick[0] * 2 * (state.a_button ? 2 : 1);
    } else if (state.left_trigger_1) {
      vel_.linear.x = state.left_stick[1] / 8 * (state.a_button ? 2 : 1);
      vel_.linear.y = -state.left_stick[0] / 8 * (state.a_button ? 2 : 1);
      vel_.angular.z = state.right_stick[0] / 2 * (state.a_button ? 2 : 1);
    } else if (state.right_trigger_1) {
      waist_hinge = state.left_stick[1] / 3.5;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(lookupLatestPosition("r_shoulder_pitch") + (right_arm_r1.value_or(0.0)));
    point.positions.push_back(lookupLatestPosition("r_shoulder_roll") + (right_arm_r2.value_or(0.0)));
    point.positions.push_back(lookupLatestPosition("l_shoulder_pitch") + (left_arm_r1.value_or(0.0)));
    point.positions.push_back(lookupLatestPosition("l_shoulder_roll") + (left_arm_r2.value_or(0.0)));
    point.positions.push_back(waist_hinge.value_or(0.0));
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    traj_.points.push_back(point);
  }

  void publishCommands() {
    traj_.header.stamp = this->now();
    traj_pub_->publish(traj_);
    cmd_vel_pub_->publish(vel_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuoriTeleop>());
  rclcpp::shutdown();
  return 0;
}
