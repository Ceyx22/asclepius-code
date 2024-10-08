#ifndef UART_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_
#define UART_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_


#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UART_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define UART_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define UART_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define UART_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef UART_HARDWARE_INTERFACE_BUILDING_DLL
#define UART_HARDWARE_INTERFACE_PUBLIC UART_HARDWARE_INTERFACE_EXPORT
#else
#define UART_HARDWARE_INTERFACE_PUBLIC UART_HARDWARE_INTERFACE_IMPORT
#endif
#define UART_HARDWARE_INTERFACE_PUBLIC_TYPE UART_HARDWARE_INTERFACE_PUBLIC
#define UART_HARDWARE_INTERFACE_LOCAL
#else
#define UART_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define UART_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define UART_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define UART_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define UART_HARDWARE_INTERFACE_PUBLIC
#define UART_HARDWARE_INTERFACE_LOCAL
#endif
#define UART_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // UART_HARDWARE_INTERFACE__VISIBLITY_CONTROL_H_
