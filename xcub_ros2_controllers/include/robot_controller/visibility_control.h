#ifndef ROBOT_CONTROLLER__VISIBILITY_CONTROL_H_
#define ROBOT_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define ROBOT_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_CONTROLLER_EXPORT __declspec(dllexport)
    #define ROBOT_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_CONTROLLER_BUILDING_LIBRARY
    #define ROBOT_CONTROLLER_PUBLIC ROBOT_CONTROLLER_EXPORT
  #else
    #define ROBOT_CONTROLLER_PUBLIC ROBOT_CONTROLLER_IMPORT
  #endif
  #define ROBOT_CONTROLLER_PUBLIC_TYPE ROBOT_CONTROLLER_PUBLIC
  #define ROBOT_CONTROLLER_LOCAL
#else
  #define ROBOT_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_CONTROLLER_PUBLIC
    #define ROBOT_CONTROLLER_LOCAL
  #endif
  #define ROBOT_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ROBOT_CONTROLLER__VISIBILITY_CONTROL_H_