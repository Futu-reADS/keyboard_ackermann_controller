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
using GearShift = tier4_external_api_msgs::msg::GearShift;
using TurnSignal = tier4_external_api_msgs::msg::TurnSignal;
using GateMode = tier4_control_msgs::msg::GateMode;

GearShiftType getUpperShift(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::DRIVE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::LOW;
  }
  if (shift == GearShift::LOW) {
    return GearShift::LOW;
  }

  return GearShift::NONE;
}

GearShiftType getLowerShift(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::LOW) {
    return GearShift::DRIVE;
  }

  return GearShift::NONE;
}

const char * getShiftName(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return "NONE";
  }
  if (shift == GearShift::PARKING) {
    return "PARKING";
  }
  if (shift == GearShift::REVERSE) {
    return "REVERSE";
  }
  if (shift == GearShift::NEUTRAL) {
    return "NEUTRAL";
  }
  if (shift == GearShift::DRIVE) {
    return "DRIVE";
  }
  if (shift == GearShift::LOW) {
    return "LOW";
  }

  return "NOT_SUPPORTED";
}

const char * getTurnSignalName(const TurnSignalType & turn_signal)
{
  if (turn_signal == TurnSignal::NONE) {
    return "NONE";
  }
  if (turn_signal == TurnSignal::LEFT) {
    return "LEFT";
  }
  if (turn_signal == TurnSignal::RIGHT) {
    return "RIGHT";
  }
  if (turn_signal == TurnSignal::HAZARD) {
    return "HAZARD";
  }

  return "NOT_SUPPORTED";
}

const char * getGateModeName(const GateModeType & gate_mode)
{
  using tier4_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }

  return "NOT_SUPPORTED";
}

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
#if 1
  while (read(STDIN_FILENO, &c, 1) > 0) {
    ;
  }
  return c;
#else
  if (read(STDIN_FILENO, &c, 1) > 0) {
    return c;
  }
  return 0;
#endif
}

void AutowareKBAckControllerNode::procKey() // const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
#if 1
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
  case '0': // case 'j': case 'k': case 'l':
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
  }

  // Change steering angle
  switch (c) {
  case 'j': // case 'u': case 'j': case 'm':
    kb_accum_state_.steer_ += steer_step_;
    if (max_steer_ < steer_ratio_*kb_accum_state_.steer_) {
      kb_accum_state_.steer_ = max_steer_/steer_ratio_;
    }
    break;
  case '0': // case 'i': case 'k': case ',':
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
#else
  last_joy_received_time_ = msg->header.stamp;
  if (joy_type_ == "G29") {
    joy_ = std::make_shared<const G29JoyConverter>(*msg);
  } else if (joy_type_ == "DS4") {
    joy_ = std::make_shared<const DS4JoyConverter>(*msg);
  } else {
    joy_ = std::make_shared<const P65JoyConverter>(*msg);
  }

  if (joy_->shift_up() || joy_->shift_down() || joy_->shift_drive() || joy_->shift_reverse()) {
    publishShift();
  }

  if (joy_->turn_signal_left() || joy_->turn_signal_right() || joy_->clear_turn_signal()) {
    publishTurnSignal();
  }

  if (joy_->gate_mode()) {
    publishGateMode();
  }

  if (joy_->autoware_engage() || joy_->autoware_disengage()) {
    publishAutowareEngage();
  }

  if (joy_->vehicle_engage() || joy_->vehicle_disengage()) {
    publishVehicleEngage();
  }

  if (joy_->emergency_stop()) {
    sendEmergencyRequest(true);
  }

  if (joy_->clear_emergency_stop()) {
    sendEmergencyRequest(false);
  }
#endif
}

void AutowareKBAckControllerNode::onVelocityReport
(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}

void AutowareKBAckControllerNode::onSteeringReport
(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
{
  steering_report_ = msg;
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
#if 0  /* temporarily disabled 20231218 */
  {
    if (!steering_report_ || ! velocity_report_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for steering_report or velocity_report msg...");
      return false;
    }

    constexpr auto timeout = 0.5;
    auto time_diff1 = this->now() - velocity_report_->header.stamp;
    if (time_diff1.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "velocity_report  is timeout");
      return false;
    }
    auto time_diff2 = this->now() - steering_report_->stamp;
    if (time_diff2.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "steering_report is timeout");
      return false;
    }
  }
#endif
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
    RCLCPP_INFO(get_logger(), "j   l
    RCLCPP_INFO(get_logger(), "  ,     <--- increase backward speed (or decrease forward speed)");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "^   ^");
    RCLCPP_INFO(get_logger(), "|   |");
    RCLCPP_INFO(get_logger(), "|   +--- 'decrease' steer angle (rotate steering wheel to the right)");
    RCLCPP_INFO(get_logger(), "|");
    RCLCPP_INFO(get_logger(), "+------- 'increase' steer angle (rotate steering wheel to the left)");
    RCLCPP_INFO(get_logger(), " ");
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
  publishExternalControlCommand();
  publishShift();
  publishHeartbeat();
}

void AutowareKBAckControllerNode::publishControlCommand()
{
  autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
  cmd.stamp = this->now();
  {
    cmd.lateral.steering_tire_angle = steer_ratio_ * kb_accum_state_.steer_;
    cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;
    cmd.longitudinal.speed = velocity_ratio_ * kb_accum_state_.lonvel_;
    cmd.longitudinal.speed =
      std::min(cmd.longitudinal.speed, static_cast<float>(max_forward_velocity_));
    cmd.longitudinal.acceleration = 0; /*
      accel_gain_wrt_velocity_diff_ *
      (cmd.longitudinal.speed - velocity_report_->longitudinal_velocity); */
  }

  pub_control_command_->publish(cmd);
  prev_control_command_ = cmd;
}

void AutowareKBAckControllerNode::publishExternalControlCommand()
{
  tier4_external_api_msgs::msg::ControlCommandStamped cmd_stamped;
  cmd_stamped.stamp = this->now();
  {
    auto & cmd = cmd_stamped.control;

#if 1
    cmd.steering_angle = steer_ratio_ * kb_accum_state_.steer_;
    cmd.steering_angle_velocity = steering_angle_velocity_;
    cmd.throttle =
      accel_ratio_ * calcMapping(kb_accum_state_.accel_, accel_sensitivity_);
    cmd.brake = brake_ratio_ * calcMapping(kb_accum_state_.brake_, brake_sensitivity_);
#else
    cmd.steering_angle = steer_ratio_ * joy_->steer();
    cmd.steering_angle_velocity = steering_angle_velocity_;
    cmd.throttle =
      accel_ratio_ * calcMapping(static_cast<double>(joy_->accel()), accel_sensitivity_);
    cmd.brake = brake_ratio_ * calcMapping(static_cast<double>(joy_->brake()), brake_sensitivity_);
#endif
  }

  pub_external_control_command_->publish(cmd_stamped);
  prev_external_control_command_ = cmd_stamped.control;
}

void AutowareKBAckControllerNode::publishShift()
{
  tier4_external_api_msgs::msg::GearShiftStamped gear_shift;
  gear_shift.stamp = this->now();
#if 1
  gear_shift.gear_shift.data = GearShift::DRIVE;
#else
  if (joy_->shift_up()) {
    gear_shift.gear_shift.data = getUpperShift(prev_shift_);
  }

  if (joy_->shift_down()) {
    gear_shift.gear_shift.data = getLowerShift(prev_shift_);
  }

  if (joy_->shift_drive()) {
    gear_shift.gear_shift.data = GearShift::DRIVE;
  }

  if (joy_->shift_reverse()) {
    gear_shift.gear_shift.data = GearShift::REVERSE;
  }
#endif
  //  RCLCPP_INFO(get_logger(), "GearShift::%s", getShiftName(gear_shift.gear_shift.data));

  pub_shift_->publish(gear_shift);
  prev_shift_ = gear_shift.gear_shift.data;

  autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
  gear_cmd.stamp = this->now();
  gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
  pub_gear_cmd_->publish(gear_cmd);
}

void AutowareKBAckControllerNode::publishTurnSignal()
{
#if 0
  tier4_external_api_msgs::msg::TurnSignalStamped turn_signal;
  turn_signal.stamp = this->now();

  if (joy_->turn_signal_left() && joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::HAZARD;
  } else if (joy_->turn_signal_left()) {
    turn_signal.turn_signal.data = TurnSignal::LEFT;
  } else if (joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::RIGHT;
  }

  if (joy_->clear_turn_signal()) {
    turn_signal.turn_signal.data = TurnSignal::NONE;
  }

  RCLCPP_INFO(get_logger(), "TurnSignal::%s", getTurnSignalName(turn_signal.turn_signal.data));

  pub_turn_signal_->publish(turn_signal);
#endif
}

void AutowareKBAckControllerNode::publishGateMode()
{
  tier4_control_msgs::msg::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::EXTERNAL;
  }

  if (prev_gate_mode_ == GateMode::EXTERNAL) {
    gate_mode.data = GateMode::AUTO;
  }

  RCLCPP_INFO(get_logger(), "GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_->publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareKBAckControllerNode::publishHeartbeat()
{
  tier4_external_api_msgs::msg::Heartbeat heartbeat;
  heartbeat.stamp = this->now();
  pub_heartbeat_->publish(heartbeat);
}

#if 0
void AutowareKBAckControllerNode::sendEmergencyRequest(bool emergency)
{
  RCLCPP_INFO(get_logger(), "%s emergency stop", emergency ? "Set" : "Clear");

  auto request = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();
  request->emergency = emergency;

  client_emergency_stop_->async_send_request(
    request, [this, emergency](
               rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture result) {
      auto response = result.get();
      if (tier4_api_utils::is_success(response->status)) {
        RCLCPP_INFO(get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}
#endif

void AutowareKBAckControllerNode::publishAutowareEngage()
{
#if 0
  auto req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
  if (joy_->autoware_engage()) {
    req->engage = true;
    RCLCPP_INFO(get_logger(), "Autoware Engage");
  }

  if (joy_->autoware_disengage()) {
    req->engage = false;
    RCLCPP_INFO(get_logger(), "Autoware Disengage");
  }

  if (!client_autoware_engage_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", client_autoware_engage_->get_service_name());
    return;
  }

  client_autoware_engage_->async_send_request(
    req, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
      RCLCPP_INFO(
        get_logger(), "%s: %d, %s", client_autoware_engage_->get_service_name(),
        result.get()->status.code, result.get()->status.message.c_str());
    });
#endif
}

void AutowareKBAckControllerNode::publishVehicleEngage()
{
  autoware_auto_vehicle_msgs::msg::Engage engage;

#if 1
  engage.engage = true;
  RCLCPP_INFO(get_logger(), "Vehicle Engage");
#else
  if (joy_->vehicle_engage()) {
    engage.engage = true;
    RCLCPP_INFO(get_logger(), "Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.engage = false;
    RCLCPP_INFO(get_logger(), "Vehicle Disengage");
  }
#endif
  
  pub_vehicle_engage_->publish(engage);
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
    flg_termio_org_set_(false)
{
  // Parameter
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
  pub_control_command_ =
    this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", qos_control_cmd);
  pub_external_control_command_ =
    this->create_publisher<tier4_external_api_msgs::msg::ControlCommandStamped>(
      "output/external_control_command", 1);
  pub_shift_ =
    this->create_publisher<tier4_external_api_msgs::msg::GearShiftStamped>("output/shift", 1);

  auto qos_gear_cmd = rclcpp::QoS(1);
  qos_gear_cmd.transient_local();
  pub_gear_cmd_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>
    ("/control/command/gear_cmd", qos_gear_cmd);

  pub_turn_signal_ = this->create_publisher<tier4_external_api_msgs::msg::TurnSignalStamped>(
    "output/turn_signal", 1);
  pub_gate_mode_ = this->create_publisher<tier4_control_msgs::msg::GateMode>("output/gate_mode", 1);
  pub_heartbeat_ =
    this->create_publisher<tier4_external_api_msgs::msg::Heartbeat>("output/heartbeat", 1);
  pub_vehicle_engage_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("output/vehicle_engage", 1);

  // Service Client
#if 1
  while (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
    rclcpp::shutdown();
    return;
  }
#else
  client_emergency_stop_ = this->create_client<tier4_external_api_msgs::srv::SetEmergency>(
  "service/emergency_stop", rmw_qos_profile_services_default, callback_group_services_);
  while (!client_emergency_stop_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for emergency_stop service connection...");
  }

  client_autoware_engage_ = this->create_client<tier4_external_api_msgs::srv::Engage>(
    "service/autoware_engage", rmw_qos_profile_services_default);
#endif

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
