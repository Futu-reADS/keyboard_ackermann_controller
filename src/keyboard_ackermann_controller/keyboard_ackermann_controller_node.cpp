#include "keyboard_ackermann_controller/keyboard_ackermann_controller.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
extern "C" {
#include <termios.h>
}

namespace keyboard_ackermann_controller
{
  //using GearShift = tier4_external_api_msgs::msg::GearShift;
  //using TurnSignal = tier4_external_api_msgs::msg::TurnSignal;
  //using GateMode = tier4_control_msgs::msg::GateMode;

double calcMapping(const double input, const double sensitivity)
{
  const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
  return std::pow(input, exponent);
}

}  // namespace

namespace keyboard_ackermann_controller
{
void AutowareKBAckControllerNode::disableKBRawMode()
{
  if (flg_termio_org_set_) {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &termio_org_);
  }
  RCLCPP_INFO(get_logger(), "disableKBRawMode() is called.");
}
void AutowareKBAckControllerNode::enableKBRawMode()
{
  RCLCPP_INFO(get_logger(), "enableKBRawMode() is called.");
  tcgetattr(STDIN_FILENO, &termio_org_);
  flg_termio_org_set_ = true;

  struct termios termio_raw = termio_org_;
  termio_raw.c_iflag &= ~(ICRNL | INPCK | ISTRIP | IXON);
  termio_raw.c_cflag |= (CS8);
  termio_raw.c_lflag &= ~(ECHO | ICANON | IEXTEN);
  termio_raw.c_cc[VMIN] = 0;
  termio_raw.c_cc[VTIME] = 0;
  int ret = tcsetattr(STDIN_FILENO, TCSANOW, &termio_raw);
  RCLCPP_INFO(get_logger(), "tcsetattr() returned %d : %s", ret, strerror(errno));
}
  
int AutowareKBAckControllerNode::pollKey()
{
  char c = 0;
  while (read(STDIN_FILENO, &c, 1) > 0) {
    ;
  }
  return c;
}

void AutowareKBAckControllerNode::procKey() // const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  int c;
  
  c = pollKey();
  if (!c) {
    return;
  }
  //RCLCPP_INFO(get_logger(), "pollKey() returned '%c'.", c);

  // Change linear velocity
  switch (c) {
  case 'i': // case 'u': case 'i':  case 'o':
    kb_accum_state_.lonvel_ += velocity_step_;
    if (max_forward_velocity_ < velocity_ratio_*kb_accum_state_.lonvel_) {
      kb_accum_state_.lonvel_ = max_forward_velocity_/velocity_ratio_;
    }
    kb_accum_state_.brake_ = 0.0;
    break;
  case 'k': // case 'j': case 'k': case 'l':
    if (velocity_step_ <= kb_accum_state_.lonvel_) {
      kb_accum_state_.lonvel_ -= velocity_step_;
    } else if (kb_accum_state_.lonvel_ <= -velocity_step_) {
      kb_accum_state_.lonvel_ += velocity_step_;
    } else {
      kb_accum_state_.lonvel_ = 0;
    }
    kb_accum_state_.brake_ = velocity_step_;
    break;
  case ',': // case 'm': case ',': case '.':
    kb_accum_state_.lonvel_ -= velocity_step_;
    if (velocity_ratio_*kb_accum_state_.lonvel_ < -max_forward_velocity_) {
      kb_accum_state_.lonvel_ = -max_forward_velocity_/velocity_ratio_;
    }
    kb_accum_state_.brake_ = 0.0;
    break;
  case ' ':
    kb_accum_state_.lonvel_ = 0;
    break;
  }

  // Change steering angle
  switch (c) {
  case 'j': // case 'u': case 'j': case 'm':
    kb_accum_state_.steer_ += steer_step_;
    if (max_steer_ < steer_ratio_*kb_accum_state_.steer_) {
      kb_accum_state_.steer_ = max_steer_/steer_ratio_;
    }
    break;
  case 'k': // case 'i': case 'k': case ',':
    if (steer_step_ <= kb_accum_state_.steer_) {
      kb_accum_state_.steer_ -= steer_step_;
    } else if (kb_accum_state_.steer_ <= -steer_step_) {
      kb_accum_state_.steer_ += steer_step_;
    } else {
      kb_accum_state_.steer_ = 0.0;
    }
    break;
  case 'l': // case 'o': case 'l': case '.':
    kb_accum_state_.steer_ += -steer_step_;
    if (steer_ratio_*kb_accum_state_.steer_ < -max_steer_) {
      kb_accum_state_.steer_ = -max_steer_/steer_ratio_;
    }
    break;
  }

  // Display current set points
  RCLCPP_INFO(get_logger(), "velocity:%lf steering angle:%lf",
	      velocity_ratio_ * kb_accum_state_.lonvel_,
	      steer_ratio_ * kb_accum_state_.steer_);
}

void AutowareKBAckControllerNode::onVelocityReport
(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "onVelocityReport() is called");
}

void AutowareKBAckControllerNode::onSteeringReport
(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
{
  steering_report_ = msg;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "onSteeringReport() is called");
}


bool AutowareKBAckControllerNode::isDataReady()
{
#if 0
  // Joy
  {
    if (!joy_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = this->now() - last_joy_received_time_;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "joy msg is timeout");
      return false;
    }
  }
#endif

  // Twist
  if (use_report_)  /* temporarily disabled for default 20231218 */
  {
    if (!steering_report_ || !velocity_report_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for steering_report or velocity_report msg...");
      return false;
    }

    double timeout = 5.0;
    double time_diff1 = now().seconds() - (double)velocity_report_->header.stamp.sec + velocity_report_->header.stamp.nanosec*1e-9;
    if (time_diff1 > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "velocity_report is expired (time diff:%lf)", time_diff1);
      return false;
    }
    double time_diff2 = now().seconds() - (double)steering_report_->stamp.sec + steering_report_->stamp.nanosec*1e-9;
    if (time_diff2 > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "steering_report is expired (time diff:%lf)", time_diff2);
      return false;
    }
  }

  return true;
}

void AutowareKBAckControllerNode::onTimer()
{
  static bool first = true;
  if (first) {
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), " ---- keyboard_ackermann_controller ---- ");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "  i     <--- increase forward speed (or decrease backward speed)");
    RCLCPP_INFO(get_logger(), "j   l");
    RCLCPP_INFO(get_logger(), "  ,     <--- increase backward speed (or decrease forward speed)");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "^   ^");
    RCLCPP_INFO(get_logger(), "|   |");
    RCLCPP_INFO(get_logger(), "|   +--- 'decrease' steer angle (rotate steering wheel to the right)");
    RCLCPP_INFO(get_logger(), "|");
    RCLCPP_INFO(get_logger(), "+------- 'increase' steer angle (rotate steering wheel to the left)");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "Press space bar to stop the vehicle.");
    RCLCPP_INFO(get_logger(), "Press Ctrl-C to quit.");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), " --------------------------------------- ");
    RCLCPP_INFO(get_logger(), " ");
    first = false;
  }

  procKey();
  if (!isDataReady()) {
    return;
  }
  publishControlCommand();
  publishMiscTopics();
}

void AutowareKBAckControllerNode::publishControlCommand()
{
  autoware_auto_control_msgs::msg::AckermannControlCommand control_cmd;
  control_cmd.stamp = this->now();
  {
    control_cmd.lateral.steering_tire_angle = steer_ratio_ * kb_accum_state_.steer_;
    control_cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;
    control_cmd.longitudinal.speed = velocity_ratio_ * abs(kb_accum_state_.lonvel_);
    float longitudinal_speed = velocity_ratio_ * kb_accum_state_.lonvel_;
    longitudinal_speed = std::min(std::max(longitudinal_speed, -static_cast<float>(max_forward_velocity_)), static_cast<float>(max_forward_velocity_));
    if (false /*use_gear_*/) {
      control_cmd.longitudinal.speed = fabs(longitudinal_speed);
    } else {
      control_cmd.longitudinal.speed = longitudinal_speed;
    }
    if (use_report_) {
      if (use_gear_) {
        control_cmd.longitudinal.acceleration = accel_gain_wrt_velocity_diff_ * (fabs(control_cmd.longitudinal.speed) - fabs(velocity_report_->longitudinal_velocity));
      } else {
        control_cmd.longitudinal.acceleration = accel_gain_wrt_velocity_diff_ * (control_cmd.longitudinal.speed - velocity_report_->longitudinal_velocity);
      }
    } else {
      control_cmd.longitudinal.acceleration = 0; /*
      accel_gain_wrt_velocity_diff_ *
      (control_cmd.longitudinal.speed - velocity_report_->longitudinal_velocity); */
    }
  }

  pub_control_command_->publish(control_cmd);
  prev_control_command_ = control_cmd;
}


void AutowareKBAckControllerNode::publishMiscTopics()
{
#if 1
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped emergency_cmd;
  emergency_cmd.stamp = now();
  emergency_cmd.emergency = false;
  pub_emergency_cmd_->publish(emergency_cmd);
#endif

  autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
  gear_cmd.stamp = now();
  if (use_gear_ && kb_accum_state_.lonvel_ < 0.0) {
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE;
  } else if (kb_accum_state_.lonvel_ == 0.0) {
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;
  } else {
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
  }
  pub_gear_cmd_->publish(gear_cmd);

  autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard_lights_cmd;
  hazard_lights_cmd.stamp = now();
  hazard_lights_cmd.command = 0;
  pub_hazard_lights_cmd_->publish(hazard_lights_cmd);

  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicators_cmd;
  turn_indicators_cmd.stamp = now();
  turn_indicators_cmd.command = 0;
  pub_turn_indicators_cmd_->publish(turn_indicators_cmd);
}


void AutowareKBAckControllerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AutowareKBAckControllerNode::onTimer, this));
}

AutowareKBAckControllerNode::AutowareKBAckControllerNode(const rclcpp::NodeOptions & node_options)
  : Node("keyboard_ackermann_controller", node_options),
    velocity_report_(NULL),
    steering_report_(NULL),
    flg_termio_org_set_(false)
{
  
  // Parameter
  use_gear_ = declare_parameter<bool>("use_gear", false);
  update_rate_ = declare_parameter<double>("update_rate", 20.0);
  accel_ratio_ = declare_parameter<double>("accel_ratio", 3.0);
  brake_ratio_ = declare_parameter<double>("brake_ratio", 5.0);
  steer_ratio_ = declare_parameter<double>("steer_ratio", 0.5);
  steering_angle_velocity_ = declare_parameter<double>("steering_angle_velocity", 0.1);
  accel_sensitivity_ = declare_parameter<double>("accel_sensitivity", 1.0);
  brake_sensitivity_ = declare_parameter<double>("brake_sensitivity", 1.0);
  velocity_gain_ = declare_parameter<double>("control_command.velocity_gain", 3.0);
  velocity_ratio_ = declare_parameter<double>("control_command.velocity_ratio", 1.0);
  max_forward_velocity_ = declare_parameter<double>("control_command.max_forward_velocity", 20.0);
  max_backward_velocity_ = declare_parameter<double>("control_command.max_backward_velocity", 3.0);
  backward_accel_ratio_ = declare_parameter<double>("control_command.backward_accel_ratio", 1.0);
  accel_gain_wrt_velocity_diff_ = declare_parameter<double>("accel_gain_wrt_velocity_diff", 1.0);

  velocity_step_ = declare_parameter<double>("control_command.velocity_step", 0.1);
  steer_step_ = declare_parameter<double>("control_command.steer_step", 0.05);
  max_steer_ = declare_parameter<double>("control_command.max_steer", 1.0);

  use_report_ = declare_parameter<bool>("use_report", true);
  RCLCPP_INFO(get_logger(), "use_report:%d", use_report_);
  
  // Callback Groups
  callback_group_subscribers_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  //sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
  //  "input/joy", 1, std::bind(&AutowareKBAckControllerNode::onJoy, this, std::placeholders::_1),
  //  subscriber_option);
  //sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
  // "input/odometry", 1,
  // std::bind(&AutowareKBAckControllerNode::onOdometry, this, std::placeholders::_1),
  // subscriber_option);
  sub_steering_report_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>
    ("/vehicle/status/steering_status", 1,
     std::bind(&AutowareKBAckControllerNode::onSteeringReport, this, std::placeholders::_1),
     subscriber_option);
  sub_velocity_report_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>
    ("/vehicle/status/velocity_status", 1,
     std::bind(&AutowareKBAckControllerNode::onVelocityReport, this, std::placeholders::_1),
     subscriber_option);
  

  // Publisher
  auto qos_control_cmd = rclcpp::QoS(1);
  qos_control_cmd.transient_local();
  pub_control_command_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos_control_cmd);
#ifdef USE_TIER4
  pub_emergency_cmd_ = create_publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("/control/command/emergency_cmd", qos_control_cmd);
#endif

  auto qos_gear_cmd = rclcpp::QoS(1);
  qos_gear_cmd.transient_local();
  pub_gear_cmd_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos_gear_cmd);
  pub_hazard_lights_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>("/control/command/hazard_lights_cmd", qos_gear_cmd);
  pub_turn_indicators_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", qos_gear_cmd);

  // Service Client
  while (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
    rclcpp::shutdown();
    return;
  }

  enableKBRawMode();

  // Timer
  initTimer(1.0 / update_rate_);
}

AutowareKBAckControllerNode::~AutowareKBAckControllerNode() {
  disableKBRawMode();
}

}  // namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(keyboard_ackermann_controller::AutowareKBAckControllerNode)
