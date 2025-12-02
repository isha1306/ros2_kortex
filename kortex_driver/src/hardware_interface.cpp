// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// You may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");
}

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
: router_tcp_{
    &transport_tcp_,
    [](k_api::KError err) { RCLCPP_ERROR_STREAM(LOGGER, "TCP callback error: " << err.toString()); }},
  session_manager_{&router_tcp_},
  router_udp_realtime_{
    &transport_udp_realtime_,
    [](k_api::KError err) { RCLCPP_ERROR_STREAM(LOGGER, "UDP callback error: " << err.toString()); }},
  session_manager_real_time_{&router_udp_realtime_},
  k_api_twist_(nullptr),
  base_{&router_tcp_},
  base_cyclic_{&router_udp_realtime_},
  gripper_motor_command_(nullptr),
  gripper_command_max_velocity_(100.0),
  gripper_command_max_force_(100.0),
  servoing_mode_hw_(k_api::Base::ServoingModeInformation()),
  joint_based_controller_running_(false),
  twist_controller_running_(false),
  gripper_controller_running_(false),
  fault_controller_running_(false),
  stop_joint_based_controller_(false),
  stop_twist_controller_(false),
  stop_gripper_controller_(false),
  stop_fault_controller_(false),
  start_joint_based_controller_(false),
  start_twist_controller_(false),
  start_gripper_controller_(false),
  start_fault_controller_(false),
  first_pass_(true),
  gripper_joint_name_(""),
  use_internal_bus_gripper_comm_(false),
  last_joint_index_(0),
  last_joint_torque_command_(std::numeric_limits<double>::quiet_NaN())
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

// ---------------------------- Initialization ----------------------------

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  std::string robot_ip = info_.hardware_parameters.at("robot_ip");
  std::string username = info_.hardware_parameters.at("username");
  std::string password = info_.hardware_parameters.at("password");
  int port = std::stoi(info_.hardware_parameters.at("port"));
  int port_realtime = std::stoi(info_.hardware_parameters.at("port_realtime"));
  int session_inactivity_timeout = std::stoi(info_.hardware_parameters.at("session_inactivity_timeout_ms"));
  int connection_inactivity_timeout = std::stoi(info_.hardware_parameters.at("connection_inactivity_timeout_ms"));
  gripper_joint_name_ = info_.hardware_parameters.at("gripper_joint_name");

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters.at("gripper_max_velocity"));
  gripper_command_max_force_ = std::stod(info_.hardware_parameters.at("gripper_max_force"));

  use_internal_bus_gripper_comm_ =
    (info_.hardware_parameters.at("use_internal_bus_gripper_comm") == "true" ||
     info_.hardware_parameters.at("use_internal_bus_gripper_comm") == "True");

  // Connect TCP/UDP
  transport_tcp_.connect(robot_ip, port);
  transport_udp_realtime_.connect(robot_ip, port_realtime);

  // Create session
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);
  create_session_info.set_connection_inactivity_timeout(connection_inactivity_timeout);

  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);

  // Reset faults & set servoing
  servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode_hw_);
  arm_mode_ = Kinova::Api::Base::SINGLE_LEVEL_SERVOING;

  try { base_.ClearFaults(); } catch (k_api::KDetailedException & ex) { RCLCPP_ERROR_STREAM(LOGGER, ex.what()); }

  servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
  arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
  base_.SetServoingMode(servoing_mode_hw_);

  // Initialize twist
  k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
  k_api_twist_command_.set_duration(0);
  k_api_twist_ = k_api_twist_command_.mutable_twist();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Resize arrays
  actuator_count_ = base_.GetActuatorCount().count();
  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(actuator_count_, integration_lvl_t::UNDEFINED);
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  twist_commands_.resize(6, 0.0);

  // Determine last joint index for torque control
  last_joint_index_ = actuator_count_ - 1;

  return CallbackReturn::SUCCESS;
}

// ---------------------------- State Interfaces ----------------------------

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_);
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_);
    }
    else
      arm_joint_names.emplace_back(info_.joints[i].name);
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    if (i == arm_joint_names.size() - 1)
      state_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]);
    else
    {
      state_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]);
      state_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]);
      state_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]);
    }
  }

  state_interfaces.emplace_back("reset_fault", "internal_fault", &in_fault_);
  return state_interfaces;
}

// ---------------------------- Command Interfaces ----------------------------

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_);
      command_interfaces.emplace_back(info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_);
      gripper_speed_command_ = gripper_command_max_velocity_;
      command_interfaces.emplace_back(info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_);
      gripper_force_command_ = gripper_command_max_force_;
    }
    else
      arm_joint_names.emplace_back(info_.joints[i].name);
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    if (i == arm_joint_names.size() - 1)
      command_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]);
    else
    {
      command_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]);
      command_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]);
      command_interfaces.emplace_back(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]);
    }
  }

  // Twist interfaces
  for (size_t i = 0; i < 6; ++i)
  {
    std::string name = (i < 3 ? "twist.linear." : "twist.angular.") + std::to_string(i % 3);
    command_interfaces.emplace_back("tcp", name, &twist_commands_[i]);
  }

  command_interfaces.emplace_back("reset_fault", "command", &reset_fault_cmd_);
  command_interfaces.emplace_back("reset_fault", "async_success", &reset_fault_async_success_);

  return command_interfaces;
}

// ---------------------------- Read & Write ----------------------------

hardware_interface::return_type KortexMultiInterfaceHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read joint states
  try
  {
    auto feedback = base_cyclic_.RefreshFeedback();
    for (size_t i = 0; i < actuator_count_; ++i)
    {
      arm_positions_[i] = feedback.actuators(i).position();
      arm_velocities_[i] = feedback.actuators(i).velocity();
      arm_efforts_[i] = feedback.actuators(i).torque();
    }

    gripper_position_ = feedback.actuators(last_joint_index_).position();
    gripper_velocity_ = feedback.actuators(last_joint_index_).velocity();
  }
  catch (const k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Read exception: " << ex.what());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KortexMultiInterfaceHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  sendJointCommands();
  sendGripperCommand();
  sendTwistCommand();
  return hardware_interface::return_type::OK;
}

// ---------------------------- Command Send Functions ----------------------------

void KortexMultiInterfaceHardware::sendJointCommands()
{
  try
  {
    k_api::Base::JointSpeeds joint_speeds;
    for (size_t i = 0; i < actuator_count_; ++i)
    {
      if (i == last_joint_index_) continue; // skip last joint for torque
      joint_speeds.add_joint(i)->set_value(arm_commands_velocities_[i]);
    }

    // Send torque for last joint
    if (!std::isnan(arm_commands_efforts_[last_joint_index_]))
    {
      k_api::Base::JointTorque joint_torque;
      joint_torque.set_joint_identifier(last_joint_index_);
      joint_torque.set_value(arm_commands_efforts_[last_joint_index_]);
      base_.SendJointTorqueCommand(joint_torque);
    }

    base_.SendJointSpeedCommand(joint_speeds);
  }
  catch (const k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Joint command exception: " << ex.what());
  }
}

void KortexMultiInterfaceHardware::sendGripperCommand()
{
  try
  {
    k_api::Base::SetServoingModeCommand servo_command;
    servo_command.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
    base_.SetServoingMode(servo_command);

    base_.SendGripperCommand(gripper_command_position_, gripper_speed_command_, gripper_force_command_);
  }
  catch (const k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Gripper command exception: " << ex.what());
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  try
  {
    k_api_twist_->mutable_linear()->set_x(twist_commands_[0]);
    k_api_twist_->mutable_linear()->set_y(twist_commands_[1]);
    k_api_twist_->mutable_linear()->set_z(twist_commands_[2]);
    k_api_twist_->mutable_angular()->set_x(twist_commands_[3]);
    k_api_twist_->mutable_angular()->set_y(twist_commands_[4]);
    k_api_twist_->mutable_angular()->set_z(twist_commands_[5]);

    base_.SendTwistCommand(k_api_twist_command_);
  }
  catch (const k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Twist command exception: " << ex.what());
  }
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
