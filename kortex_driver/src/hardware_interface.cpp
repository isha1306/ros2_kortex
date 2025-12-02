// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
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
/*!\file
 *
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
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
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_{&router_tcp_},
  router_udp_realtime_{
    &transport_udp_realtime_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
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
  use_internal_bus_gripper_comm_(false)
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
  }
  // Username to log into the robot controller
  std::string username = info_.hardware_parameters["username"];
  if (username.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  // Password to log into the robot controller
  std::string password = info_.hardware_parameters["password"];
  if (password.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  int port = std::stoi(info_.hardware_parameters["port"]);
  if (port <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Port used '%d'", port);
  }
  int port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime);
  }

  int session_inactivity_timeout =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  int connection_inactivity_timeout =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }
  // gripper joint name
  gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters["gripper_max_velocity"]);
  gripper_command_max_force_ = std::stod(info_.hardware_parameters["gripper_max_force"]);

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  // connections
  transport_tcp_.connect(robot_ip, port);
  transport_udp_realtime_.connect(robot_ip, port_realtime);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);  // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(
    connection_inactivity_timeout);  // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating session for communication");
  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);
  RCLCPP_INFO(LOGGER, "Session created");

  // reset faults on activation, go back to low level servoing after
  {
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = Kinova::Api::Base::SINGLE_LEVEL_SERVOING;

    try
    {
      base_.ClearFaults();
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }

    // low level servoing on startup
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
    arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
    base_.SetServoingMode(servoing_mode_hw_);
  }

  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    k_api_twist_command_.set_duration(0);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  actuator_count_ = base_.GetActuatorCount().count();
  RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(
    actuator_count_, integration_lvl_t::UNDEFINED);  // start in undefined
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  twist_commands_.resize(6, 0.0);

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!(info_.joints[i].state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          info_.joints[i].state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          info_.joints[i].state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", info_.joints[i].name.c_str(),
        info_.joints[i].state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  if (
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    if (i == arm_joint_names.size() - 1) // last joint torque controlled
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
    }
    else
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
    }
  }

  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_));
      gripper_speed_command_ = gripper_command_max_velocity_;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_));
      gripper_force_command_ = gripper_command_max_force_;
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    if (i == arm_joint_names.size() - 1) // last joint torque controlled
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
    else
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "reset_fault", "async_success", &reset_fault_async_success_));

  return command_interfaces;
}

// Remaining code stays unchanged (prepare_command_mode_switch, perform_command_mode_switch,
// on_activate, on_deactivate, read, write, sendJointCommands, sendGripperCommand,
// sendTwistCommand, etc.)  

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
