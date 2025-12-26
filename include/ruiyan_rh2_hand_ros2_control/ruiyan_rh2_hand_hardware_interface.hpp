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
#pragma once
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ruiyan_rh2_hand_ros2_control/ryhandlib.h"
#include "ruiyan_rh2_hand_ros2_control/visibility_control.hpp"

namespace ruiyan_rh2_hand_ros2_control
{

class RuiyanRH2HandHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RuiyanRH2HandHardwareInterface)

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Static callback functions
  static s8_t hand_bus_write(CanMsg_t stuMsg);
  static void hand_call_back(CanMsg_t stuMsg, void * para);

private:
  static std::atomic<bool> hand_exists_;  // Don't allow two hand controllers in a single ROS2 node
  static std::mutex hand_data_mutex_;     // Mutex for hand data access
  static RyCanServoBus_t stuServoCan_;
  static CanMsg_t stuListenMsg_[40];
  static ServoData_t sutServoDataW_[15];
  static ServoData_t sutServoDataR_[15];
  static volatile s16_t uwTick_;

  // Communication parameters
  static int sock_;            // CAN Socket
  std::string can_interface_;  // e.g., "can0"
  struct sockaddr_can addr_;   // CAN Address
  struct ifreq ifr_;           // Network interface request
  int hand_speed_;             // Hand movement speed
  int current_limit_;          // Current limit for servos
  std::atomic<bool> is_running_{false};
  std::thread read_thread_;

  // Hand specifications
  static constexpr uint8_t NUM_JOINTS = 6;
  static constexpr uint8_t NUM_MOTORS = 6;
  static constexpr uint16_t SERVO_CMD_MAX = 4095;
  static constexpr uint16_t DEFAULT_SPEED = 1000;
  static constexpr uint16_t DEFAULT_CURRENT_LIMIT = 300;

  // Joint data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // State tracking
  bool first_read_completed_;

  // Joint limits (radians) - from URDF analysis
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;

  // Helper functions for communication
  bool connect_to_hand();
  void disconnect_from_hand();
  bool read_joint_commands(std::vector<double> & positions);
  bool write_joint_commands(const std::vector<double> & commands);
  bool open_can_socket();
  void close_can_socket();
  bool init_servo_can_system();
  static bool send_can_message(u8_t id, u8_t * data, u8_t len);
  bool receive_can_message(struct can_frame * frame);

  // Utility functions
  void enforce_joint_limits();
  double clamp_to_limits(double value, size_t joint_index) const;
  double cmd_to_radx(int cmd, double radmax);
  int radx_to_cmd(double rad, double radmax);
};

// C wrapper function declarations for Ryhandlib usage
extern "C" {
s8_t bus_write_wrapper(CanMsg_t stuMsg);
void call_back_wrapper(CanMsg_t stuMsg, void * para);
}

}  // namespace ruiyan_rh2_hand_ros2_control