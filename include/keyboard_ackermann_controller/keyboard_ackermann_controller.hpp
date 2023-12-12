#ifndef KEYBOARD_ACKERMANN_CONTROLLER_HPP_
#define KEYBOARD_ACKERMANN_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/msg/turn_signal_stamped.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>

#include <algorithm>
#include <memory>
#include <string>
extern "C" {
#include <termios.h>
}

namespace keyboard_ackermann_controller
{
using GearShiftType = tier4_external_api_msgs::msg::GearShift::_data_type;
using TurnSignalType = tier4_external_api_msgs::msg::TurnSignal::_data_type;
using GateModeType = tier4_control_msgs::msg::GateMode::_data_type;

struct KeyboardAccumState {
  double accel_;
  double brake_;
  double lonvel_;
  double steer_;
  KeyboardAccumState() : accel_(0), brake_(0), lonvel_(0), steer_(0) {}
};
  
class AutowareKBAckControllerNode : public rclcpp::Node
{
public:
  explicit AutowareKBAckControllerNode(const rclcpp::NodeOptions & node_options);
  ~AutowareKBAckControllerNode();
  
private:
  // Parameter
  //std::string joy_type_;
  double update_rate_;
  double accel_ratio_;
  double brake_ratio_;
  double steer_ratio_;
  double steering_angle_velocity_;
  double accel_sensitivity_;
  double brake_sensitivity_;

  // ControlCommand Parameter
  double velocity_gain_;
  double max_forward_velocity_;
  double max_backward_velocity_;
  double backward_accel_ratio_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscriber
  //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_report_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_report_;

  //rclcpp::Time last_joy_received_time_;
  rclcpp::Time last_key_received_time_;
  //std::shared_ptr<const JoyConverterBase> joy_;
  KeyboardAccumState kb_accum_state_;
  //geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
  autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocity_report_;
  autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr steering_report_;

  bool flg_termio_org_set_;
  struct termios termio_org_;
  void disableKBRawMode();
  void enableKBRawMode();

  int pollKey();
  void procKey();
  //void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  //void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg);
  void onSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    pub_control_command_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_external_control_command_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::GearShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::TurnSignalStamped>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr pub_vehicle_engage_;

  

  void publishControlCommand();
  void publishExternalControlCommand();
  void publishShift();
  void publishTurnSignal();
  void publishGateMode();
  void publishHeartbeat();
  void publishAutowareEngage();
  void publishVehicleEngage();
  //void sendEmergencyRequest(bool emergency);

  // Service Client
  rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr client_emergency_stop_;
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr client_autoware_engage_;

  // Previous State
  autoware_auto_control_msgs::msg::AckermannControlCommand prev_control_command_;
  tier4_external_api_msgs::msg::ControlCommand prev_external_control_command_;
  GearShiftType prev_shift_ = tier4_external_api_msgs::msg::GearShift::NONE;
  TurnSignalType prev_turn_signal_ = tier4_external_api_msgs::msg::TurnSignal::NONE;
  GateModeType prev_gate_mode_ = tier4_control_msgs::msg::GateMode::AUTO;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  bool isDataReady();
  void onTimer();
};
}  // namespace

#endif  // 
