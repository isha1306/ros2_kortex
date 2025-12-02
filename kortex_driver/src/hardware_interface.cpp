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

// ---------- on_init(), export_state_interfaces(), export_command_interfaces() 
// remain unchanged from your original code ----------

// ---------- write(), read(), readGripperPosition(), etc. remain unchanged ----------

// Updated sendJointCommands() with torque control for last wrist joint
void KortexMultiInterfaceHardware::sendJointCommands()
{
  incrementId();

  // If torque control active for last joint
  if (joint_based_controller_running_ && actuator_count_ > 0)
  {
    size_t last = actuator_count_ - 1;  // last joint
    auto command = k_api::BaseCyclic::Command();
    command = base_cyclic_.Refresh();  // copy previous feedback

    // set torque for last joint
    command.mutable_actuators(last)->set_command_id(last + 1);
    command.mutable_actuators(last)->set_control_mode(k_api::Actuator::ControlMode::TORQUE);
    command.mutable_actuators(last)->set_torque_joint(static_cast<float>(arm_commands_efforts_[last]));

    // set zero torque for other joints
    for (size_t i = 0; i < actuator_count_; i++)
    {
      if (i == last) continue;
      command.mutable_actuators(i)->set_control_mode(k_api::Actuator::ControlMode::TORQUE);
      command.mutable_actuators(i)->set_torque_joint(0.0f);
    }

    // send command
    try
    {
      feedback_ = base_cyclic_.Refresh(command);
    }
    catch (k_api::KDetailedException & ex)
    {
      feedback_ = base_cyclic_.RefreshFeedback();
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());
    }
    catch (std::exception & ex_std)
    {
      feedback_ = base_cyclic_.RefreshFeedback();
      RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
    }
  }
  else
  {
    // default position/velocity control
    prepareCommands();
    try
    {
      feedback_ = base_cyclic_.Refresh(base_command_);
    }
    catch (...)
    {
      feedback_ = base_cyclic_.RefreshFeedback();
    }
  }
}

// Updated prepareCommands() for hybrid control
void KortexMultiInterfaceHardware::prepareCommands()
{
  if (actuator_count_ == 0) return;
  size_t last = actuator_count_ - 1;  // last joint for torque

  for (size_t i = 0; i < actuator_count_; i++)
  {
    if (i == last)
      continue;  // skip last joint, torque handled separately

    // position/velocity control for other joints
    float cmd_degrees_tmp_ =
      static_cast<float>(KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
    float cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));

    base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
    // velocity not fully supported in API yet
  }
}

// ---------- sendTwistCommand(), sendGripperCommand(), incrementId() remain unchanged ----------

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
