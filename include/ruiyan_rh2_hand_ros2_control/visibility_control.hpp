#pragma once

#ifndef RUIYAN_RH2_HAND_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
#define RUIYAN_RH2_HAND_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef RUIYAN_RH2_HAND_ROS2_CONTROL_BUILDING_LIBRARY
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC RUIYAN_RH2_HAND_ROS2_CONTROL_EXPORT
  #else
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC RUIYAN_RH2_HAND_ROS2_CONTROL_IMPORT
  #endif
  #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC_TYPE RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
  #define RUIYAN_RH2_HAND_ROS2_CONTROL_LOCAL
#else
  #define RUIYAN_RH2_HAND_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define RUIYAN_RH2_HAND_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC
    #define RUIYAN_RH2_HAND_ROS2_CONTROL_LOCAL
  #endif
  #define RUIYAN_RH2_HAND_ROS2_CONTROL_PUBLIC_TYPE
#endif

#endif  // RUIYAN_RH2_HAND_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_