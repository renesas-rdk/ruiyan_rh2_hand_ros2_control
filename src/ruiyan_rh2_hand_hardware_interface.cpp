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

// Static member definitions
std::atomic<bool> RuiyanRH2HandHardwareInterface::hand_exists_{false};
std::mutex RuiyanRH2HandHardwareInterface::hand_data_mutex_;
RyCanServoBus_t RuiyanRH2HandHardwareInterface::stuServoCan_{};
CanMsg_t RuiyanRH2HandHardwareInterface::stuListenMsg_[40]{};
ServoData_t RuiyanRH2HandHardwareInterface::sutServoDataW_[15]{};
ServoData_t RuiyanRH2HandHardwareInterface::sutServoDataR_[15]{};
volatile s16_t RuiyanRH2HandHardwareInterface::uwTick_{0};
int RuiyanRH2HandHardwareInterface::sock_{-1};

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Don't allow multiple hand instances on the same ROS2 node
  {
    if (hand_exists_.exchange(true)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "Only one instance of RuiyanRH2HandHardwareInterface is allowed per ROS2 node");
      return CallbackReturn::ERROR;
    }
  }

  // Initialize parameters
  // Parse can_interface parameter
  can_interface_ = info_.hardware_parameters.at("can_interface");
  if (can_interface_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Parameter 'can_interface' is required but not specified");
    hand_exists_.store(false);
    return CallbackReturn::ERROR;
  }

  // Parse speed parameter (default: 1000)
  auto it = info_.hardware_parameters.find("hand_speed");
  if (it != info_.hardware_parameters.end()) {
    try {
      hand_speed_ = std::stoi(it->second);
      if (hand_speed_ < 0 || hand_speed_ > 5000) {
        RCLCPP_WARN(
          rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
          "Invalid speed value: %d. Speed should be between 0-5000. Using default value 1000",
          hand_speed_);
        hand_speed_ = DEFAULT_SPEED;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "Failed to parse speed parameter: %s. Using default value 1000", e.what());
      hand_speed_ = DEFAULT_SPEED;
    }
  }

  // Parse current limit parameter (default: 100)
  current_limits_.resize(NUM_MOTORS);
  it = info_.hardware_parameters.find("current_limits");
  if (it != info_.hardware_parameters.end()) {
    // Parse space-separated string
    std::istringstream iss(it->second);
    std::vector<std::string> tokens;
    std::string token;

    while (iss >> token) {
      tokens.push_back(token);
    }

    if (tokens.size() == NUM_MOTORS) {
      for (size_t i = 0; i < NUM_MOTORS; ++i) {
        current_limits_[i] = std::stoi(tokens[i]);
      }
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "Invalid current_limits array size: %zu (expected %d)", tokens.size(), NUM_MOTORS);
      hand_exists_.store(false);
      return CallbackReturn::ERROR;
    }
  } else {
    // Default values
    current_limits_ = {300, 300, 300, 300, 300, 300};
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
    "Using CAN interface: %s, hand speed: %d, current limits: [%s]", can_interface_.c_str(),
    hand_speed_,
    std::accumulate(
      current_limits_.begin(), current_limits_.end(), std::string(),
      [](const std::string & a, double b) {
        return a + (a.length() > 0 ? ", " : "") + std::to_string(b);
      })
      .c_str());

  // Verify we have the expected number of joints
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Expected %d joints, but got %zu",
      NUM_JOINTS, info_.joints.size());
    hand_exists_.store(false);
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

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Configuring...");

  if (!init_servo_can_system()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to configure RH2 hand hardware servo system.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Successfully configured");

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

  // Set running flag
  is_running_.store(true);

  if (!connect_to_hand()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to connect to RH2 hand");
    is_running_.store(false);
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

  // Shutdown running flag
  is_running_.store(false);

  // Join read thread
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  // Disconnect from hand
  disconnect_from_hand();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Cleaning up...");

  // Clean up servo CAN system
  free(stuServoCan_.pstuHook);
  free(stuServoCan_.pstuListen);
  stuServoCan_.pstuHook = nullptr;
  stuServoCan_.pstuListen = nullptr;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RuiyanRH2HandHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Shutting down...");

  // Remove instance flag
  hand_exists_.store(false);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RuiyanRH2HandHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read current joint positions and currents from the hand
  std::vector<double> current_positions(NUM_JOINTS);
  if (!read_joint_commands(current_positions)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Failed to read joint information");
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

// Low-level communication driver for Ruiyan RH2 hand
bool RuiyanRH2HandHardwareInterface::connect_to_hand()
{
  // Open CAN socket
  if (!open_can_socket()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to open CAN socket on interface: %s", can_interface_.c_str());
    return false;
  }

  // Create thread for receiving CAN messages
  read_thread_ = std::thread([this]() {
    // Start receiving loop
    while (is_running_) {
      // Place the work to be performed by the high-priority thread here
      auto now = std::chrono::system_clock::now();
      auto now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
      uwTick_ = static_cast<s16_t>(now_ms % 1000);

      struct can_frame frame;
      if ((sock_ >= 0) && receive_can_message(&frame)) {
        // Convert received message to CanMsg format
        CanMsg_t received_msg;
        memset(&received_msg, 0, sizeof(CanMsg_t));
        received_msg.ulId = frame.can_id;
        received_msg.ucLen = frame.can_dlc;
        memcpy(received_msg.pucDat, frame.data, frame.can_dlc);

        // Call handler function
        RyCanServoLibRcvMsg(&stuServoCan_, received_msg);
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  // Clear error
  RyParam_ClearFault(&stuServoCan_, 0, 1);

  return true;
}

void RuiyanRH2HandHardwareInterface::disconnect_from_hand()
{
  // Stop CAN interface
  close_can_socket();
}

bool RuiyanRH2HandHardwareInterface::read_joint_commands(std::vector<double> & positions)
{
  // Read current position from servo data
  std::lock_guard<std::mutex> lock(hand_data_mutex_);
  for (int i = 0; i < NUM_MOTORS; i++) {
    positions[i] = cmd_to_radx(sutServoDataR_[i].stuInfo.ub_P, joint_max_limits_.at(i));
  }

  return true;
}

bool RuiyanRH2HandHardwareInterface::write_joint_commands(const std::vector<double> & /*commands*/)
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Set target position
    sutServoDataW_[i].stuCmd.usTp =
      static_cast<u16_t>(radx_to_cmd(hw_commands_[i], joint_max_limits_.at(i)));

    // Send command to servo
    RyMotion_ServoMove_Mix(
      &stuServoCan_, static_cast<u8_t>(i + 1), sutServoDataW_[i].stuCmd.usTp,
      sutServoDataW_[i].stuCmd.usTv, sutServoDataW_[i].stuCmd.usTc, &sutServoDataR_[i], 1);
  }

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

// CAN protocol related functions

bool RuiyanRH2HandHardwareInterface::open_can_socket()
{
  // Create SocketCAN socket
  if ((sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Error while opening CAN socket: %s",
      std::strerror(errno));
    return false;
  }

  // Get interface index
  const char * can_interface_name = can_interface_.c_str();
  strcpy(ifr_.ifr_name, can_interface_name);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr_) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to get interface index for %s: %s", can_interface_name, std::strerror(errno));
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  // Bind the socket to the CAN interface
  if (::bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to bind CAN socket to interface %s: %s", can_interface_name, std::strerror(errno));
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  int flags = fcntl(sock_, F_GETFL, 0);
  if (flags < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Error getting socket flags: %s",
      std::strerror(errno));
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Error setting socket to non-blocking mode: %s", std::strerror(errno));
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
    "Successfully opened CAN socket on interface: %s", can_interface_name);

  int buf_size;
  socklen_t len = sizeof(buf_size);

  // Get receive buffer size
  getsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &buf_size, &len);
  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Receive buffer size: %d bytes",
    buf_size);

  // Get send buffer size
  getsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &buf_size, &len);
  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Send buffer size: %d bytes", buf_size);

  return true;
}

void RuiyanRH2HandHardwareInterface::close_can_socket()
{
  // Check if socket is valid before attempting to close
  if (sock_ < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "CAN socket is already closed or invalid");
    return;
  }

  // Close the CAN socket
  if (::close(sock_) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "Error closing CAN socket: %s",
      std::strerror(errno));
    sock_ = -1;
    return;
  }

  // Reset sock_ to invalid value
  sock_ = -1;
  RCLCPP_INFO(
    rclcpp::get_logger("RuiyanRH2HandHardwareInterface"), "CAN socket closed successfully");
}

bool RuiyanRH2HandHardwareInterface::send_can_message(u8_t id, u8_t * data, u8_t len)
{
  struct can_frame frame;

  // Set CAN frame content
  frame.can_id = id;             // Frame ID
  frame.can_dlc = len;           // Data length
  for (int i = 0; i < len; i++)  // Data content
  {
    frame.data[i] = data[i];
  }

  // Send CAN frame
  if (::write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to send CAN message with ID 0x%X: %s", id, std::strerror(errno));
    return false;
  }

  return true;
}

bool RuiyanRH2HandHardwareInterface::receive_can_message(struct can_frame * frame)
{
  // Receive CAN frame
  if (::read(sock_, frame, sizeof(struct can_frame)) < 0) {
    // No data available or error
    return false;
  }

  return true;
}

bool RuiyanRH2HandHardwareInterface::init_servo_can_system()
{
  // Reset stuServoCan content
  memset(&stuServoCan_, 0, sizeof(RyCanServoBus_t));

  // Specify the maximum number of Hooks supported.
  // This should be determined by the user based on actual application needs.
  // It's recommended to set a value greater than 2 (at least one Hook is needed per bus;
  // if the user doesn't specify, the library will request at least one Hook internally).
  stuServoCan_.usHookNum = 5;

  // Apply for and specify the required Hook data space.
  // The following two lines of operation are optional for the user;
  // RyCanServoBusInit will automatically apply, but the program stack must be sufficient.
  stuServoCan_.pstuHook = (MsgHook_t *)malloc(stuServoCan_.usHookNum * sizeof(MsgHook_t));
  memset(stuServoCan_.pstuHook, 0, stuServoCan_.usHookNum * sizeof(MsgHook_t));

  // Specify the maximum number of listeners supported. This should be determined by the user based on actual application needs.
  // If the servo motor active reporting function is required, sufficient listeners must be provided, with one listener needed per servo motor.
  stuServoCan_.usListenNum = 31 + 1;
  // Apply for and specify the required listen data space.
  // The following two lines of operation are optional for the user;
  // RyCanServoBusInit will automatically apply, but the program stack must be sufficient.
  stuServoCan_.pstuListen = (MsgListen_t *)malloc(stuServoCan_.usListenNum * sizeof(MsgListen_t));
  memset(stuServoCan_.pstuListen, 0, stuServoCan_.usListenNum * sizeof(MsgListen_t));

  // Initialize the library; it will use malloc internally,
  // so ensure there's enough stack space. Please check the stack space settings.
  if (RyCanServoBusInit(&stuServoCan_, bus_write_wrapper, (volatile u16_t *)&uwTick_, 1000) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
      "Failed to initialize RyCanServoBusInit.");
    return false;
  }

  // Add one listener per (servo reply CAN ID + CMD).
  // We listen to CMD=0xAA (mix motion feedback) and CMD=0xA0 (GetServoInfo replies) for each servo.
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    stuListenMsg_[i].ulId = SERVO_BACK_ID(i + 1);
    stuListenMsg_[i].pucDat[0] = 0xAA;

    if (AddListen(&stuServoCan_, &stuListenMsg_[i], call_back_wrapper) == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "AddListen failed (CMD=0xAA) for servo_id=%u (rx_id=0x%X).", static_cast<unsigned>(i + 1),
        static_cast<unsigned>(stuListenMsg_[i].ulId));
      return false;
    }
  }

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    stuListenMsg_[NUM_MOTORS + i].ulId = SERVO_BACK_ID(i + 1);
    stuListenMsg_[NUM_MOTORS + i].pucDat[0] = 0xA0;

    if (AddListen(&stuServoCan_, &stuListenMsg_[NUM_MOTORS + i], call_back_wrapper) == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RuiyanRH2HandHardwareInterface"),
        "AddListen failed (CMD=0xA0) for servo_id=%u (rx_id=0x%X).", static_cast<unsigned>(i + 1),
        static_cast<unsigned>(stuListenMsg_[NUM_MOTORS + i].ulId));
      return false;
    }
  }

  // Init default servo command
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    sutServoDataW_[i].pucDat[0] = 0xaa;  // CMD_SET_TARGET_POS2
    sutServoDataW_[i].stuCmd.usTp = SERVO_CMD_MAX;
    sutServoDataW_[i].stuCmd.usTv = static_cast<u16_t>(hand_speed_);
    sutServoDataW_[i].stuCmd.usTc = static_cast<u16_t>(current_limits_[i]);
  }

  return true;
}

s8_t RuiyanRH2HandHardwareInterface::hand_bus_write(CanMsg_t stuMsg)
{
  // 0: success, other: fail
  s8_t ret = 0;

  if (sock_ >= 0) {
    if (send_can_message(static_cast<u8_t>(stuMsg.ulId), stuMsg.pucDat, stuMsg.ucLen))
      ret = 0;
    else
      ret = -1;
  } else {
    ret = -1;
  }

  return ret;
}

void RuiyanRH2HandHardwareInterface::hand_call_back(CanMsg_t stuMsg, void * para)
{
  // Extract motor/servo node ID from the CAN frame ID.
  // In this protocol, the reply frame ID is (servo_id + 0x100). By casting ulId to u8_t,
  // we keep only the low 8 bits, which effectively yields the original servo_id.
  // Example: ulId = 0x101 (257) -> id = 0x01 (servo 1).
  u8_t id = static_cast<u8_t>(stuMsg.ulId);
  (void)para;

#if 1
  // For testing purposes, if any motor is found to be in an error state, it can be handled here.
  if (stuMsg.pucDat[1] == enServo_CurrentOverE) {
    // Handle errors
    RyParam_ClearFault(&stuServoCan_, id, 1);
  }
#endif

  // Collect motor data
  switch (stuMsg.pucDat[0]) {
    case 0xa0:  // CMD_GET_MOTOR_INFO
    case 0xa1:  // CMD_SET_TARGET_POS
    case 0xa6:  // CMD_SET_MOTOR_PWM
    case 0xa9:  // CMD_SET_TARGET_CURRENT
    case 0xaa:  // CMD_SET_TARGET_POS2
      if (id && (id < 0x10)) {
        std::lock_guard<std::mutex> lock(hand_data_mutex_);
        sutServoDataR_[id - 1] = *(ServoData_t *)stuMsg.pucDat;
      }
      break;

    default:
      break;
  }
}

// Helper function to convert the command value to radians
double RuiyanRH2HandHardwareInterface::cmd_to_radx(int cmd, double radmax)
{
  if (cmd < 0) {
    cmd = 0;
  } else if (cmd > SERVO_CMD_MAX) {
    cmd = SERVO_CMD_MAX;
  }
  return radmax * cmd / SERVO_CMD_MAX;
}

int RuiyanRH2HandHardwareInterface::radx_to_cmd(double rad, double radmax)
{
  if (rad < 0) {
    rad = 0;
  } else if (rad > radmax) {
    rad = radmax;
  }
  return (int)(rad * SERVO_CMD_MAX / radmax);
}

// C wrapper implementations
extern "C" {
s8_t bus_write_wrapper(CanMsg_t stuMsg)
{
  return RuiyanRH2HandHardwareInterface::hand_bus_write(stuMsg);
}

void call_back_wrapper(CanMsg_t stuMsg, void * para)
{
  RuiyanRH2HandHardwareInterface::hand_call_back(stuMsg, para);
}
}

}  // namespace ruiyan_rh2_hand_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruiyan_rh2_hand_ros2_control::RuiyanRH2HandHardwareInterface, hardware_interface::SystemInterface)