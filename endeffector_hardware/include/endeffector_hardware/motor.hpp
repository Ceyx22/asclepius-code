// #ifndef ENDEFFECTOR_HARDWARE__MOTOR_HPP_
// #define ENDEFFECTOR_HARDWARE__MOTOR_HPP_

// #include <iostream>
// #include <cmath>
// #include <dynamixel_sdk.h>
// #include "rclcpp/rclcpp.hpp"

// #define PROTOCOL_VERSION = 1.0
// #define DXL_MIN_POSITION_VALUE = 0           // Minimum position value
// #define DXL_MAX_POSITION_VALUE = 1023        // Maximum position value
// #define DXL_MIN_ANGLE = 0.0                  // Minimum angle in radians
// #define DXL_MAX_ANGLE = 300.0 * M_PI / 180.0 // Maximum angle in radians
// class Motor
// {
//     enum ADDRESS
//     {
//         ADDR_TORQUE_ENABLE = 24,       // Address to enable/disable the torque
//         ADDR_GOAL_POSITION = 30,       // Address to set the goal position
//         ADDR_PRESENT_POSITION = 36,    // Address to get the present position
//         ADDR_MOVING_SPEED = 32,        // Address to set the moving speed
//         ADDR_PRESENT_SPEED = 38,       // Address to get the present speed
//         ADDR_PRESENT_LOAD = 40,        // Address to get the present load
//         ADDR_PRESENT_VOLTAGE = 42,     // Address to get the present voltage
//         ADDR_PRESENT_TEMPERATURE = 43, // Address to get the present temperature
//         ADDR_CW_ANGLE_LIMIT = 6,       // Clockwise angle limit
//         ADDR_CCW_ANGLE_LIMIT = 8,      // Counter-clockwise angle limit
//         ADDR_MAX_TORQUE = 14,          // Maximum torque
//     };

// public:
//     int32_t read_position(uint8_t id, dynamixel::PacketHandler *packet, dynamixel::PortHandler *port)
//     {
//         // uint8_t dxl_error = 0;
//         // uint16_t dxl_present_position = 0;
//         int dxl_comm_result = packet->read2ByteTxRx(port, id, ADDRESS::ADDR_PRESENT_POSITION, &dxl_present_position, &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to read position for ID %d: %s", id, packet->getTxRxResult(dxl_comm_result));
//             return -1;
//         }
//         else if (dxl_error != 0)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error reading position for ID %d: %s", id, packet->getRxPacketError(dxl_error));
//             return -1;
//         }

//         return dxl_present_position;
//     }

// private:
//     uint8_t dxl_error = 0;
//     uint16_t dxl_present_position = 0;
//     // Limits for AX-12A
// };

// #endif // ENDEFFECTOR_HARDWARE__MOTOR_HPP_
