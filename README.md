# Ruiyan RH2 Hand ROS2 Control Package

# Ruiyan RH2 Hand ROS2 Control Package

ROS 2 package that provides a ros2_control hardware interface for the Ruiyan RH2 6-DOF dexterous hand. This package contains the hardware interface implementation that communicates with the physical hand hardware via CAN bus using the RyCAN servo protocol.

## Features

- ros2_control hardware interface plugin (exported via `hardware_interface_plugin.xml`)
- Hardware interface implementation for CAN bus communication with RyCAN servo protocol
- Force-position mixed control support for safe manipulation tasks
- Contact detection and adaptive current limiting for delicate object handling
- ros2_control URDF macro for hardware interface integration
- Support for both CAN and CANFD communication protocols

## Related Packages

- **ruiyan_rh2_hand_bringup**: Contains launch files, controller configurations, robot URDF descriptions, and test scripts for operating the hand
- **ruiyan_rh2_hand_description**: Contains the hand's visual and collision meshes, joint definitions

## Package Layout

- `include/` / `src/`: Hardware interface implementation (`ruiyan_rh2_hand_hardware_interface`)
- `lib/`: Compiled shared library for the Ruiyan RH2 Hand.
- `urdf/`: ros2_control URDF macro for hardware interface integration
- `hardware_interface_plugin.xml`: Plugin description file for the hardware interface

## Prerequisites

- ROS 2 (Jazzy or newer) with `ros2_control` ecosystem
- A colcon workspace (e.g., `~/ros2_ws`)
- In case of using with the physical hand hardware:
  - Ruiyan RH2 Hand
  - CAN or CANFD interface support for hardware communication
  - A power supply unit (PSU) capable of providing sufficient current for the hand servos, recommended 24V 10A for optimal performance

**Important**: If your PSU cannot provide sufficient current, consider reducing the hand current limit parameter to avoid undervoltage issues.
Each RyCAN servo can draw up to the configured current limit under load.

## Usage
This package provides the hardware interface that should be used with the `ruiyan_rh2_hand_bringup` package for launching and controlling the hand.

To use this hardware interface in your hand system, you'll typically launch one of the bringup configurations:

```bash
# For joint position control
ros2 launch ruiyan_rh2_hand_bringup ruiyan_rh2_hand_joint_position_control.launch.py

# For joint trajectory control
ros2 launch ruiyan_rh2_hand_bringup ruiyan_rh2_hand_joint_trajectory_control.launch.py
```

After launching, you can introspect the hardware interface:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## URDF Integration
The ros2_control hardware interface macro is provided in `urdf/`:
- `ruiyan_rh2_hand_macro.ros2_control.xacro`: ros2_control hardware interface, transmissions, and interfaces

To include the hardware interface in your hand URDF:
```xml
<xacro:include filename="$(find ruiyan_rh2_hand_ros2_control)/urdf/ruiyan_rh2_hand_macro.ros2_control.xacro"/>
<xacro:ruiyan_rh2_hand_ros2_control
    name="ruiyan_rh2_hand"
    can_interface="can2"
    hand_speed="2000"
    current_limits="300 300 300 300 300 300"
    use_mock_hardware="false"
    prefix="$(arg prefix)"
/>
```
Adjust arguments as needed (see the xacro file for available parameters).

## Configuration Options
The hardware interface supports the following configuration options:
- `can_interface`: CAN device path for hardware communication (e.g., "can2")
- `hand_speed`: Hand movement speed in hardware library unit (default: "1000")
- `current_limits`: Space-separated list of current limits for each joint in hardware unit (default: "300 300 300 300 300 300")
- `use_mock_hardware`: Set to "true" for simulation/testing without physical hardware

## Joint Interfaces
The hardware interface exposes 6 joints with position and velocity states:
- `thumb_proximal_yaw_joint`, `thumb_proximal_pitch_joint`
- `index_proximal_joint`, `middle_proximal_joint`, `ring_proximal_joint`, `pinky_proximal_joint`

## Development
For detailed usage examples, launch configurations, controller setups, and test scripts, see the `ruiyan_rh2_hand_bringup` package.