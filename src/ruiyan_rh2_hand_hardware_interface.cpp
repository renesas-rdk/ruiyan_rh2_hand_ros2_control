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
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Initialize parameters
  // Parse can_interface parameter
  can_interface_name_ = info_.hardware_parameters.at("can_interface");
  if (can_interface_name_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Parameter 'can_interface' is required but not specified");
    return CallbackReturn::ERROR;
  }
  // Parse speed parameter (default: 1000)
  speed_ = DEFAULT_SPEED;
  auto it = info_.hardware_parameters.find("speed");
  if (it != info_.hardware_parameters.end()) {
    try {
      speed_ = std::stoi(it->second);
      if (speed_ < 1 || speed_ > 5000) {
        RCLCPP_WARN(
          rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
          "Invalid speed value: %d. Speed should be between 1-100. Using default value 50", speed_);
        speed_ = DEFAULT_SPEED;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "Failed to parse speed parameter: %s. Using default value 50", e.what());
      speed_ = DEFAULT_SPEED;
    }
  }

  // Define the CAN frame callback
  auto callback = [this](agilex::piper::CanFrameMsg & frame) { this->parse_can_frame(frame); };

  can_interface_ =
    std::make_unique<agilex::piper::CanInterface>(can_interface_name_, 1000000, callback);

  // RCLCPP_INFO(
  //   rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Initialized with Hand ID: %d", hand_id_);

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
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to connect to RH2 hand");
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
  if (!open_can_socket()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to open CAN socket on interface: %s", can_interface_name_.c_str());
    return false;
  }

  if (!init_servo_can_system()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to open CAN socket on interface: %s", can_interface_name_.c_str());
    return false;
  }
  RCLCPP_WARN(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "TODO: Implement connect_to_hand()");

  // For now, simulate successful connection
  return true;
}

float RuiyanRH2HandHardwareInterface::cmd_to_radx(int cmd, float radmax)
{
  if (cmd < 0) {
    cmd = 0;
  } else if (cmd > SERVO_CMD_MAX) {
    cmd = SERVO_CMD_MAX;
  }
  return radmax * cmd / SERVO_CMD_MAX;
}

int RuiyanRH2HandHardwareInterface::radx_to_cmd(float rad, float radmax)
{
  if (rad < 0) {
    rad = 0;
  } else if (rad > radmax) {
    rad = radmax;
  }
  return (int)(rad * SERVO_CMD_MAX / radmax);
}

double RuiyanRH2HandHardwareInterface::evaluatePolynomial(
  double coefficients[], int degree, double x)
{
  double result = 0;

  for (int i = 0; i <= degree; ++i) {
    result += coefficients[degree - i] * pow(x, i);
  }

  return result;
}

void RuiyanRH2HandHardwareInterface::parse_can_frame(agilex::piper::CanFrameMsg & frame)
{
  // Convert agilex::piper::CanFrameMsg to CanMsg_t
  // TODO: Because using the can_interface from agilex_piper_controller, need to be apdapted to Ruiyan's CanMsg_t
  CanMsg_t stuMsg;

  stuMsg.ulId = frame.arbitration_id;
  stuMsg.ucLen = frame.dlc;
  std::memcpy(stuMsg.pucDat, frame.data, frame.dlc);

  //decode the CAN message using Ruiyan RH2 hand CAN protocol
  RyCanServoLibRcvMsg(&stuServoCan, stuMsg);
}

bool RuiyanRH2HandHardwareInterface::init_servo_can_system()
{
  // Reset stuServoCan content
  memset(&stuServoCan, 0, sizeof(RyCanServoBus_t));

  // Specify the maximum number of Hooks supported.
  // This should be determined by the user based on actual application needs.
  // It's recommended to set a value greater than 2 (at least one Hook is needed per bus;
  // if the user doesn't specify, the library will request at least one Hook internally).
  stuServoCan.usHookNum = 5;

  // Apply for and specify the required Hook data space.
  // The following two lines of operation are optional for the user;
  // RyCanServoBusInit will automatically apply, but the program stack must be sufficient.
  stuServoCan.pstuHook = (MsgHook_t *)malloc(stuServoCan.usHookNum * sizeof(MsgHook_t));
  memset(stuServoCan.pstuHook, 0, stuServoCan.usHookNum * sizeof(MsgHook_t));

  // Specify the maximum number of listeners supported. This should be determined by the user based on actual application needs.
  // If the servo motor active reporting function is required, sufficient listeners must be provided, with one listener needed per servo motor.
  stuServoCan.usListenNum = 31 + 1;
  // Apply for and specify the required listen data space.
  // The following two lines of operation are optional for the user;
  // RyCanServoBusInit will automatically apply, but the program stack must be sufficient.
  stuServoCan.pstuListen = (MsgListen_t *)malloc(stuServoCan.usListenNum * sizeof(MsgListen_t));
  memset(stuServoCan.pstuListen, 0, stuServoCan.usListenNum * sizeof(MsgListen_t));

  // Initialize the library; it will use malloc internally,
  // so ensure there's enough stack space. Please check the stack space settings.
  if (RyCanServoBusInit(&stuServoCan, BusWrite, (volatile u16_t *)&uwTick, 1000) != 0) {
    return false;
  }

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    stuListenMsg[i].ulId = SERVO_BACK_ID(i + 1);
    stuListenMsg[i].pucDat[0] = 0xAA;

    if (AddListen(&stuServoCan, &stuListenMsg[i], CallBck0) != 0) {
      return false;
    }
  }

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    stuListenMsg[NUM_MOTORS + i].ulId = SERVO_BACK_ID(i + 1);
    stuListenMsg[NUM_MOTORS + i].pucDat[0] = 0xA0;

    if (AddListen(&stuServoCan, &stuListenMsg[NUM_MOTORS + i], CallBck0) != 0) {
      return false;
    }
  }

  // ---- Init default servo command ----
  sutServoDataW[0].pucDat[0] = 0xaa;
  sutServoDataW[0].stuCmd.usTp = SERVO_CMD_MAX;
  sutServoDataW[0].stuCmd.usTv = DEFAULT_SPEED;
  sutServoDataW[0].stuCmd.usTc = 80;
  for (uint8_t i = 1; i < NUM_MOTORS; ++i) {
    sutServoDataW[i] = sutServoDataW[0];
  }
  return true;
}

s8_t RuiyanRH2HandHardwareInterface::BusWrite(CanMsg_t stuMsg)
{
  return bus_send_message(stuMsg) ? 0 : -1;
}

void RuiyanRH2HandHardwareInterface::CallBck0(CanMsg_t stuMsg, void * para)
{
  u8_t id = stuMsg.ulId;
  (void)para;
#if 1

  // For testing purposes, if any motor is found to be in an error state, it can be handled here.
  if (stuMsg.pucDat[1] == enServo_CurrentOverE) {
    // Handle errors
    RyParam_ClearFault(&stuServoCan, id, 1);
  }

#endif

  // Collect motor data
  switch (stuMsg.pucDat[0]) {
    case 0xa0:
    case 0xa1:
    case 0xa6:
    case 0xa9:
    case 0xaa:
      if (id && (id < 0x10)) sutServoDataR[id - 1] = *(ServoData_t *)stuMsg.pucDat;
      break;

    default:
      break;
  }
}

bool RuiyanRH2HandHardwareInterface::bus_send_message(const CanMsg_t & msg)
{
  return can_interface_->send_message(msg.ulId, msg.pucDat, msg.ucLen);
}

bool RuiyanRH2HandHardwareInterface::open_can_socket()
{
  if (!can_interface_->initialize()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to initialize CAN interface: %s", std::strerror(errno));
    return false;
  }

  // Start the CAN interface
  if (!can_interface_->start()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to start CAN interface: %s",
      std::strerror(errno));
    return false;
  }

  return true;
}

void RuiyanRH2HandHardwareInterface::disconnect_from_hand()
{
  // TODO: Implement actual disconnection from RH2 hand
  // This should properly close communication channels and cleanup resources

  // Stop CAN interface
  if (can_interface_) {
    can_interface_->stop();
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to stop CAN interface: %s",
      std::strerror(errno));
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "CAN interface stopped successfully");
  }

  free(stuServoCan.pstuHook);
  free(stuServoCan.pstuListen);
  stuServoCan.pstuHook = nullptr;
  stuServoCan.pstuListen = nullptr;
}

bool RuiyanRH2HandHardwareInterface::read_joint_positions(std::vector<double> & positions)
{
  // TODO: Implement actual reading of joint positions from RH2 hand
  positions.assign(6, 0.0);  // j_ang size

  for (const auto & map : joint_motor_map) {
    int p = sutServoDataR[map.motor_id].stuInfo.ub_P;
    // Unused
    // int v = sutServoDataR[map.motor_id].stuInfo.ub_V;
    // int t = sutServoDataR[map.motor_id].stuInfo.ub_I;
    // if (v > 2047) v -= 4096;
    // if (t > 2047) t -= 4096;

    // motor_id → joint_master
    positions[map.joint_id] = cmd_to_radx(p, map.offset_deg * M_PI / 180.0);
  }

  // // For now, simulate joint positions (all at mid-range)
  // for (size_t i = 0; i < NUM_JOINTS; ++i) {
  //   positions[i] = (joint_min_limits_[i] + joint_max_limits_[i]) / 2.0;
  // }

  return true;
}

void RuiyanRH2HandHardwareInterface::UpdataMotor(void)
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    RyMotion_ServoMove_Mix(
      &stuServoCan, i + 1, sutServoDataW[i].stuCmd.usTp, sutServoDataW[i].stuCmd.usTv,
      sutServoDataW[i].stuCmd.usTc, &sutServoDataR[i], 1);
  }
}

bool RuiyanRH2HandHardwareInterface::write_joint_commands(const std::vector<double> & /*commands*/)
{
  // TODO: Implement actual writing of joint commands to RH2 hand
  // Set default motor parameters
  for (int i = 0; i < NUM_MOTORS; i++) {
    sutServoDataW[i].stuCmd.usTp = SERVO_CMD_MAX;
    sutServoDataW[i].stuCmd.usTv = speed_;
    sutServoDataW[i].stuCmd.usTc = 1000;
  }
  for (const auto & map : joint_motor_map) {
    // Joint angle in radians (from ros2_control command interface)
    double joint_rad = hw_commands_[map.joint_id];

    // Convert joint angle to servo command with mechanical offset
    sutServoDataW[map.motor_id].stuCmd.usTp = radx_to_cmd(joint_rad, map.offset_deg * M_PI / 180.0);
  }
  UpdataMotor();
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