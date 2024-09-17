
#ifndef ENDEFFECTOR_HARDWARE__VISIBLITY_CONTROL_H_
#define ENDEFFECTOR_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ENDEFFECTOR_HARDWARE_EXPORT __attribute__((dllexport))
#define ENDEFFECTOR_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define ENDEFFECTOR_HARDWARE_EXPORT __declspec(dllexport)
#define ENDEFFECTOR_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef ENDEFFECTOR_HARDWARE_BUILDING_DLL
#define ENDEFFECTOR_HARDWARE_PUBLIC ENDEFFECTOR_HARDWARE_EXPORT
#else
#define ENDEFFECTOR_HARDWARE_PUBLIC ENDEFFECTOR_HARDWARE_IMPORT
#endif
#define ENDEFFECTOR_HARDWARE_PUBLIC_TYPE ENDEFFECTOR_HARDWARE_PUBLIC
#define ENDEFFECTOR_HARDWARE_LOCAL
#else
#define ENDEFFECTOR_HARDWARE_EXPORT __attribute__((visibility("default")))
#define ENDEFFECTOR_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define ENDEFFECTOR_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define ENDEFFECTOR_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define ENDEFFECTOR_HARDWARE_PUBLIC
#define ENDEFFECTOR_HARDWARE_LOCAL
#endif
#define ENDEFFECTOR_HARDWARE_PUBLIC_TYPE
#endif

#endif  // ENDEFFECTOR_HARDWARE__VISIBLITY_CONTROL_H_