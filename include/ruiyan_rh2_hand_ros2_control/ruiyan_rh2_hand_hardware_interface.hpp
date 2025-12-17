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
  // int hand_id_;
  std::string can_interface_name_;  // e.g., "can0"
  int speed_;

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
  static constexpr uint8_t NUM_MOTORS = 6;
  static constexpr uint16_t SERVO_CMD_MAX = 4095;
  static constexpr uint16_t DEFAULT_SPEED = 1000;
  // Joint → Motor mapping table
  //
  // This table defines how IK joint angles (model space) are mapped to
  // physical motors (actuator space) in an underactuated dexterous hand.
  //
  // Key ideas:
  // 1) Joint ≠ Motor
  //    - The real hand has fewer motors.
  //    - One motor may drive multiple joints via mechanical coupling.
  //
  // 2) Only "master joints" are directly actuated by motors.
  //    - Coupled/slave joints are computed from the master joint
  //      (e.g. via polynomial coupling) and MUST NOT be sent to hardware.
  //
  // 3) This mapping is used in the hardware interface:
  //    - Convert it to a motor command using radx_to_cmd()
  //    - Send the command only to the corresponding motor_id
  struct JointMotorMap
  {
    int motor_id;       // hardware motor index
    int joint_id;       // j_ang index (0..10)
    double offset_deg;  // mechanical zero offset
  };

  static constexpr JointMotorMap joint_motor_map[NUM_MOTORS] = {
    {0, 0, 135.0},  // Thumb base
    {1, 1, 40.0},   // Index MCP
    {2, 2, 87.0},   // Middle MCP
    {3, 3, 90.0},   // Ring MCP
    {4, 4, 90.0},   // Pinky MCP
    {5, 5, 88.5}    // Thumb MCP
  };

  // static constexpr std::array<std::array<double, 4>, 5> poly_coeff = {{
  //   {{0.000329, -0.035054, 2.558963, 0.272863}},   // Thumb
  //   {{0.000010, -0.004996, 1.426094, -0.044273}},  // Index
  //   {{0.000002, -0.002910, 1.283182, -0.088568}},  // Middle
  //   {{0.000010, -0.004996, 1.426094, -0.044273}},  // Ring
  //   {{0.000016, -0.006612, 1.529302, -0.011082}}   // Pinky
  // }};

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
  void UpdataMotor(void);
  float cmd_to_radx(int cmd, float radmax);
  int radx_to_cmd(float rad, float radmax);
  double evaluatePolynomial(double coefficients[], int degree, double x);

  // Utility functions
  void enforce_joint_limits();
  double clamp_to_limits(double value, size_t joint_index) const;
};

}  // namespace ruiyan_rh2_hand_ros2_control