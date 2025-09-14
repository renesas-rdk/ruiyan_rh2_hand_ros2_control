// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************

#include "ruiyan_rh2_hand_ros2_control/ruiyan_rh2_hand_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ruiyan_rh2_hand_ros2_control
{

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Initialize hand ID parameter
  hand_id_ = std::stoi(
    info_.hardware_parameters.count("hand_id") ? info_.hardware_parameters.at("hand_id") : "1");

  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Initialized with Hand ID: %d", hand_id_);

  // Verify we have the expected number of joints
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Expected %d joints, but got %zu",
      NUM_JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Initialize joint vectors
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);

  // Initialize state tracking
  first_read_completed_ = false;

  // Initialize joint limits from URDF/XACRO definitions
  joint_min_limits_.resize(info_.joints.size());
  joint_max_limits_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // Extract limits from URDF joint info
    const auto & joint = info_.joints[i];
    joint_min_limits_[i] = std::stod(joint.command_interfaces[0].min);
    joint_max_limits_[i] = std::stod(joint.command_interfaces[0].max);

    RCLCPP_INFO(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Joint %s: min=%.3f rad (%.1f deg), max=%.3f rad (%.1f deg)", joint.name.c_str(),
      joint_min_limits_[i], joint_min_limits_[i] * 180.0 / M_PI, joint_max_limits_[i],
      joint_max_limits_[i] * 180.0 / M_PI);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RuiyanRH2HandHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RuiyanRH2HandHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Activating...");

  if (!connect_to_hand()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to connect to RH2 hand (Hand ID: %d)", hand_id_);
    return CallbackReturn::ERROR;
  }

  // Reset state tracking - let first read() call initialize positions
  first_read_completed_ = false;

  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Successfully activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Deactivating...");
  disconnect_from_hand();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RuiyanRH2HandHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read current joint positions from the hand
  std::vector<double> current_positions(NUM_JOINTS);
  if (!read_joint_positions(current_positions)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to read joint positions");
    return hardware_interface::return_type::ERROR;
  }

  // Store previous positions for velocity calculation
  std::vector<double> prev_positions = hw_positions_;

  // Update positions
  hw_positions_ = current_positions;

  // Calculate joint velocities using finite difference
  if (first_read_completed_ && period.seconds() > 0.0) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions[i]) / period.seconds();
    }
  } else {
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  }

  // Initialize commands to current positions on first read
  if (!first_read_completed_) {
    std::copy(hw_positions_.begin(), hw_positions_.end(), hw_commands_.begin());
    first_read_completed_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "First read completed - initialized positions and commands");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RuiyanRH2HandHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Enforce joint limits on commands
  enforce_joint_limits();

  // Write joint commands to the hand
  if (!write_joint_commands(hw_commands_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to write joint commands");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// TODO: Implement low-level communication driver for Ruiyan RH2 hand
bool RuiyanRH2HandHardwareInterface::connect_to_hand()
{
  // TODO: Implement actual connection to RH2 hand
  // This should establish communication via the specified interface (USB, CAN, etc.)
  // Example implementations might include:
  // - Serial/USB communication setup
  // - CAN bus initialization
  // - Network socket connection
  // - Proprietary protocol initialization

  RCLCPP_WARN(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
    "TODO: Implement connect_to_hand() for Hand ID: %d", hand_id_);

  // For now, simulate successful connection
  return true;
}

void RuiyanRH2HandHardwareInterface::disconnect_from_hand()
{
  // TODO: Implement actual disconnection from RH2 hand
  // This should properly close communication channels and cleanup resources

  RCLCPP_WARN(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "TODO: Implement disconnect_from_hand()");
}

bool RuiyanRH2HandHardwareInterface::read_joint_positions(std::vector<double> & positions)
{
  // TODO: Implement actual reading of joint positions from RH2 hand

  // For now, simulate joint positions (all at mid-range)
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    positions[i] = (joint_min_limits_[i] + joint_max_limits_[i]) / 2.0;
  }

  return true;
}

bool RuiyanRH2HandHardwareInterface::write_joint_commands(const std::vector<double> & /*commands*/)
{
  // TODO: Implement actual writing of joint commands to RH2 hand

  return true;
}

void RuiyanRH2HandHardwareInterface::enforce_joint_limits()
{
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = clamp_to_limits(hw_commands_[i], i);
  }
}

double RuiyanRH2HandHardwareInterface::clamp_to_limits(double value, size_t joint_index) const
{
  return std::max(joint_min_limits_[joint_index], std::min(joint_max_limits_[joint_index], value));
}

}  // namespace ruiyan_rh2_hand_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruiyan_rh2_hand_ros2_control::RuiyanRH2HandHardwareInterface, hardware_interface::SystemInterface)