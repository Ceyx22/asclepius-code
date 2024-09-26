#ifndef ENDEFFECTOR_HARDWARE__COMMS_HPP_
#define ENDEFFECTOR_HARDWARE__COMMS_HPP_

#include <iostream>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

// #define PROTOCOL_VERSION = 1.0
// #define DXL_MIN_POSITION_VALUE = 0           // Minimum position value
// #define DXL_MAX_POSITION_VALUE = 1023        // Maximum position value
// #define DXL_MIN_ANGLE = 0.0                  // Minimum angle in radians
// #define DXL_MAX_ANGLE = 300.0 * M_PI / 180.0 // Maximum angle in radians
namespace endeffector_hardware
{
    class Comms
    {

    public:
        // Helper
        bool connect(const std::string &portname, int32_t baud_rate);
        void disconnect();

        bool write2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t value);
        bool write1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t value);
        bool read2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t *data);

    private:
        dynamixel::PortHandler *portHandler_;
        dynamixel::PacketHandler *packetHandler_;

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
    };
}
#endif // ENDEFFECTOR_HARDWARE__COMMS_HPP_
