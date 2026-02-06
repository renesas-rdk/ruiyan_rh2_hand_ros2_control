cmake_minimum_required(VERSION 3.8)
project(ruiyan_rh2_hand_ros2_control)

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Detect platform and select appropriate library
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(RUIYAN_LIB_NAME "libRyhand64.so")
  message(STATUS "Detected x86_64 platform - using libRyhand64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(RUIYAN_LIB_NAME "libRyhandArm64.so")
  message(STATUS "Detected ARM64 platform - using libRyhandArm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  set(RUIYAN_LIB_NAME "libRyhandArm.so")
  message(STATUS "Detected ARM 32-bit platform - using libRyhandArm.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
  set(RUIYAN_LIB_NAME "libRyhand.so")
  message(STATUS "Detected x86 32-bit platform - using libRyhand.so")
else()
  message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(RUIYAN_SRC_LIB ${LIB_DIR}/${RUIYAN_LIB_NAME})

if(NOT EXISTS ${RUIYAN_SRC_LIB})
  message(FATAL_ERROR "Library not found: ${RUIYAN_SRC_LIB}")
endif()

message(STATUS "Using library: ${RUIYAN_SRC_LIB}")

# Create an imported target for the vendor library
add_library(ruiyan_vendor SHARED IMPORTED)
set_target_properties(ruiyan_vendor PROPERTIES
  IMPORTED_LOCATION ${RUIYAN_SRC_LIB}
  IMPORTED_NO_SONAME TRUE
)

# Hardware interface library
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/ruiyan_rh2_hand_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_hardware_interface
  PUBLIC
    ruiyan_vendor
)

ament_target_dependencies(${PROJECT_NAME}_hardware_interface
  PUBLIC
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    pluginlib
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the vendor library
install(
  FILES ${RUIYAN_SRC_LIB}
  DESTINATION lib
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config and urdf files
install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()