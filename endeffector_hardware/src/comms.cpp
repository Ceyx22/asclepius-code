#include "endeffector_hardware/comms.hpp"
#include "rclcpp/rclcpp.hpp"

// Protocol version
#define PROTOCOL_VERSION 1.0

#define LOGGER_IDENTIFIER "endeffector_hardware > comms"

namespace endeffector_hardware
{
    bool Comms::connect(const std::string &device, int32_t baud_rate)
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(device.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        dxl_comm_result = portHandler_->openPort();
        if (dxl_comm_result == false)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to open the port!");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to open the port.");
        }

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = portHandler_->setBaudRate(baud_rate);
        if (dxl_comm_result == false)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set the baudrate!");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set the baudrate.");
        }

        return true;
    }

    bool Comms::write2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t value)
    {
        dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, dynamixel_id, address, value, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to write 2 bytes.");
            return false;
        }
        return true;
    };

    bool Comms::write1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t value)
    {

        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dynamixel_id, address, value, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to write 1 bytes.");
            return false;
        }
        return true;
    };

    bool Comms::read2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t *data)
    {
        dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, dynamixel_id, address, data, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read 2 bytes.");
            return false;
        }
        return true;
    };

    void Comms::disconnect()
    {
        portHandler_->closePort();

        delete packetHandler_;
        delete portHandler_;
    }

}