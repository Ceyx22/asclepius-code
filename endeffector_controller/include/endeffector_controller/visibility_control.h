#ifndef ENDEFFECTOR_CONTROLLER__VISIBILITY_CONTROL_H_
#define ENDEFFECTOR_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ENDEFFECTOR_CONTROLLER_EXPORT __attribute__((dllexport))
#define ENDEFFECTOR_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define ENDEFFECTOR_CONTROLLER_EXPORT __declspec(dllexport)
#define ENDEFFECTOR_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef ENDEFFECTOR_CONTROLLER_BUILDING_DLL
#define ENDEFFECTOR_CONTROLLER_PUBLIC ENDEFFECTOR_CONTROLLER_EXPORT
#else
#define ENDEFFECTOR_CONTROLLER_PUBLIC ENDEFFECTOR_CONTROLLER_IMPORT
#endif
#define ENDEFFECTOR_CONTROLLER_PUBLIC_TYPE ENDEFFECTOR_CONTROLLER_PUBLIC
#define ENDEFFECTOR_CONTROLLER_LOCAL
#else
#define ENDEFFECTOR_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define ENDEFFECTOR_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define ENDEFFECTOR_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define ENDEFFECTOR_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define ENDEFFECTOR_CONTROLLER_PUBLIC
#define ENDEFFECTOR_CONTROLLER_LOCAL
#endif
#define ENDEFFECTOR_CONTROLLER_PUBLIC_TYPE
#endif

#endif // ENDEFFECTOR_CONTROLLER__VISIBILITY_CONTROL_H_