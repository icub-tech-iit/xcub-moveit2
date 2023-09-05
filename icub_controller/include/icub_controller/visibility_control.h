#ifndef ICUB_CONTROLLER__VISIBILITY_CONTROL_H_
#define ICUB_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ICUB_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define ICUB_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define ICUB_CONTROLLER_EXPORT __declspec(dllexport)
    #define ICUB_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ICUB_CONTROLLER_BUILDING_LIBRARY
    #define ICUB_CONTROLLER_PUBLIC ICUB_CONTROLLER_EXPORT
  #else
    #define ICUB_CONTROLLER_PUBLIC ICUB_CONTROLLER_IMPORT
  #endif
  #define ICUB_CONTROLLER_PUBLIC_TYPE ICUB_CONTROLLER_PUBLIC
  #define ICUB_CONTROLLER_LOCAL
#else
  #define ICUB_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define ICUB_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define ICUB_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define ICUB_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ICUB_CONTROLLER_PUBLIC
    #define ICUB_CONTROLLER_LOCAL
  #endif
  #define ICUB_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ICUB_CONTROLLER__VISIBILITY_CONTROL_H_