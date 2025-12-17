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
#include <array>
#include <cstdint>
#include <vector>

#include "agilex_piper_controller/can_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ruiyan_rh2_hand_ros2_control/visibility_control.hpp"
#include "ryhandlib.h"
#include "stdbool.h"

extern "C" {
#include "ryhandlib.h"
}
#ifdef OK
#undef OK
#endif

#ifdef ERROR
#undef ERROR
#endif

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
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Communication parameters
  int hand_id_;
  std::string can_interface_name_;  // e.g., "can0"

  static std::unique_ptr<agilex::piper::CanInterface> can_interface_;  // CAN interface

  static RyCanServoBus_t stuServoCan;
  static CanMsg_t stuListenMsg[40];
  static ServoData_t sutServoDataW[15];
  static ServoData_t sutServoDataR[15];
  volatile s16_t uwTick;

  static void CallBck0(CanMsg_t stuMsg, void * para);
  static s8_t BusWrite(CanMsg_t stuMsg);
  static bool bus_send_message(const CanMsg_t & msg);

  // Hand specifications
  static constexpr uint8_t NUM_JOINTS = 6;

  // Joint data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // State tracking
  bool first_read_completed_;

  // Joint limits (radians) - from URDF analysis
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;

  // Helper functions for communication (TODO: implement low-level driver)
  bool connect_to_hand();
  void disconnect_from_hand();
  bool read_joint_positions(std::vector<double> & positions);
  bool write_joint_commands(const std::vector<double> & commands);
  bool open_can_socket();
  bool init_servo_can_system();
  void parse_can_frame(agilex::piper::CanFrameMsg & frame);

  // Utility functions
  void enforce_joint_limits();
  double clamp_to_limits(double value, size_t joint_index) const;
};

}  // namespace ruiyan_rh2_hand_ros2_control