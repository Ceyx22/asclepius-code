#include <cmath>
#include <string>
#include <algorithm>
#include <iostream>

#include "endeffector_hardware/comms.hpp"
#include "endeffector_hardware/motor.hpp"
#include "rclcpp/rclcpp.hpp"

// EEPROM AREA - Stored permanently and requires a reboot to take effect
#define ADDR_MODEL_NUMBER_L 0       // 2 bytes
#define ADDR_MODEL_NUMBER_H 1       // 2 bytes
#define ADDR_FIRMWARE_VERSION 2     // 1 byte
#define ADDR_ID 3                   // 1 byte
#define ADDR_BAUD_RATE 4            // 1 byte
#define ADDR_RETURN_DELAY_TIME 5    // 1 byte
#define ADDR_CW_ANGLE_LIMIT 6       // 2 bytes (Clockwise Angle Limit)
#define ADDR_CCW_ANGLE_LIMIT 8      // 2 bytes (Counterclockwise Angle Limit)
#define ADDR_DRIVE_MODE 10          // 1 byte
#define ADDR_LIMIT_TEMPERATURE 11   // 1 byte
#define ADDR_LOW_VOLTAGE_LIMIT 12   // 1 byte
#define ADDR_HIGH_VOLTAGE_LIMIT 13  // 1 byte
#define ADDR_MAX_TORQUE 14          // 2 bytes
#define ADDR_STATUS_RETURN_LEVEL 16 // 1 byte
#define ADDR_ALARM_LED 17           // 1 byte
#define ADDR_ALARM_SHUTDOWN 18      // 1 byte

// RAM AREA - Temporary and can change during runtime
#define ADDR_TORQUE_ENABLE 24          // 1 byte (Enable Torque: 1 = On, 0 = Off)
#define ADDR_LED 25                    // 1 byte (LED On/Off)
#define ADDR_CW_COMPLIANCE_MARGIN 26   // 1 byte
#define ADDR_CCW_COMPLIANCE_MARGIN 27  // 1 byte
#define ADDR_CW_COMPLIANCE_SLOPE 28    // 1 byte
#define ADDR_CCW_COMPLIANCE_SLOPE 29   // 1 byte
#define ADDR_GOAL_POSITION 30          // 2 bytes (Goal Position)
#define ADDR_MOVING_SPEED 32           // 2 bytes (Moving Speed)
#define ADDR_TORQUE_LIMIT 34           // 2 bytes (Torque Limit)
#define ADDR_PRESENT_POSITION 36       // 2 bytes (Current Position)
#define ADDR_PRESENT_SPEED 38          // 2 bytes (Current Speed)
#define ADDR_PRESENT_LOAD 40           // 2 bytes (Current Load)
#define ADDR_PRESENT_VOLTAGE 42        // 1 byte (Current Voltage)
#define ADDR_PRESENT_TEMPERATURE 43    // 1 byte (Current Temperature)
#define ADDR_REGISTERED_INSTRUCTION 44 // 1 byte
#define ADDR_MOVING 46                 // 1 byte (Moving: 1 = Moving, 0 = Stopped)
#define ADDR_LOCK 47                   // 1 byte (Lock EEPROM: 1 = Locked, 0 = Unlocked)
#define ADDR_PUNCH 48                  // 2 bytes (Punch)

#define MIN_MOVING_SPEED_VALUE 80
#define MAX_MOVING_SPEED_VALUE 1023
#define STOP 0
#define REVERSE_OFFSET 1024
#define MAX_REV_PER_MIN 114 // 60

#define DXL_MOTOR_POSITION_OFFSET 512
#define DXL_MAX_POSITION_VALUE 1023
#define DXL_MIN_POSITION_VALUE 0
#define DXL_MAX_ANGLE 300
#define DXL_MIN_ANGLE 0
#define LOGGER_IDENTIFIER "endeffector_hardware > motor"

const double SPEED_CONVERSION_FACTOR = MAX_MOVING_SPEED_VALUE / (MAX_REV_PER_MIN * 2 * M_PI / 60.0);

namespace endeffector_hardware
{
    void Motor::init(const std::string name, const uint8_t motor_id)
    {
        this->name = name;
        this->motor_id_ = motor_id;
        // this->inverted_ = is_inverted ? -1 : 1;

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "JOINT: %s ", name.c_str());
    }

    bool Motor::setup(const Comms &connection)
    {
        connection_ = connection;

        bool is_success = true;

        // Enable Torque of DYNAMIXEL
        is_success = connection_.write1ByteTxRx(motor_id_, ADDR_TORQUE_ENABLE, 1);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to enable torque.");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to enable torque.");
        }

        return true;
    }

    void Motor::shutdown()
    {
        // Disable Torque of DYNAMIXEL
        bool is_success = connection_.write1ByteTxRx(motor_id_, ADDR_TORQUE_ENABLE, 0);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to disable torque.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to disable torque.");
        }
    }

    bool Motor::write_velocity(double velocity)
    {

        uint16_t cur_dynamixel_vel = convert_to_motor_vel(vel);
        uint16_t dynamixel_vel = convert_to_motor_vel(velocity);

        if (dynamixel_vel != cur_dynamixel_vel)
        {
            bool is_success = connection_.write2ByteTxRx(motor_id_, ADDR_MOVING_SPEED, dynamixel_vel);
            if (!is_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set Wheel rotation.");
                return false;
            }
            else
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set Wheel rotation. %u", dynamixel_vel);
            }
        }

        vel = convert_to_real_vel(dynamixel_vel);

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "%s: requested=%f executed=%f", name.c_str(), velocity, vel);

        return true;
    }

    bool Motor::write_position(double position)
    {

        uint16_t cur_dynamixel_pos = convert_to_motor_pos(pos);
        uint16_t dynamixel_pos = convert_to_motor_pos(position);
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "current_pos: %d, new position: %d", cur_dynamixel_pos, dynamixel_pos);

        if (dynamixel_pos != cur_dynamixel_pos)
        {
            bool is_success = connection_.write2ByteTxRx(motor_id_, ADDR_GOAL_POSITION, dynamixel_pos);
            if (!is_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set joint position.");
                return false;
            }
            else
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set joint position. %u", dynamixel_pos);
            }
        }

        pos = convert_to_angle(position);

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "%s: requested=%f executed=%f", name.c_str(), position, pos);

        return true;
    }

    double Motor::get_velocity()
    {
        return vel;
        // uint16_t dynamixel_vel;

        // bool is_success = connection_.read2ByteTxRx(dynamixel_id_, ADDR_PRESENT_SPEED, &dynamixel_vel);
        // if (!is_success)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read Wheel rotation.");
        //     return vel;
        // }
        // else
        // {
        //     RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to read Wheel rotation. %u", dynamixel_vel);
        // }

        // return convert_to_ros_control_vel(dynamixel_vel);
    }

    bool Motor::update_position()
    {
        uint16_t dynamixel_pos;
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Reading position for id %d: ", motor_id_);
        bool is_success = connection_.read2ByteTxRx(motor_id_, ADDR_PRESENT_SPEED, &dynamixel_pos);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read position for id %d: ", motor_id_);
            return false;
        }
        // else
        // {
        //     RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to read position. %u", dynamixel_pos);
        // }
        pos = convert_to_angle(dynamixel_pos);
        return true;
        // return pos;
    }

    uint16_t Motor::convert_to_motor_vel(double rad_vel)
    {
        int16_t dynamixel_vel = static_cast<int>(std::round(SPEED_CONVERSION_FACTOR * rad_vel));
        dynamixel_vel = std::clamp(dynamixel_vel, static_cast<int16_t>(-1 * MAX_MOVING_SPEED_VALUE), static_cast<int16_t>(MAX_MOVING_SPEED_VALUE));
        int16_t dynamixel_value = dynamixel_vel;

        if (std::abs(dynamixel_value) < MIN_MOVING_SPEED_VALUE)
        {
            dynamixel_value = 0;
        }

        if (dynamixel_value < 0)
        {
            dynamixel_value = REVERSE_OFFSET - dynamixel_value;
        }

        return static_cast<uint16_t>(dynamixel_value);
    }

    double Motor::convert_to_real_vel(uint16_t dynamixel_vel)
    {
        int16_t dynamixel_value = static_cast<int16_t>(dynamixel_vel);

        if (dynamixel_value >= REVERSE_OFFSET)
        {
            dynamixel_value = REVERSE_OFFSET - dynamixel_value;
        }

        if (std::abs(dynamixel_value) < MIN_MOVING_SPEED_VALUE)
        {
            dynamixel_value = 0;
        }

        return (dynamixel_value) / SPEED_CONVERSION_FACTOR;
    }
    uint16_t Motor::convert_to_motor_pos(double command_position)
    {
        return static_cast<uint16_t>(DXL_MOTOR_POSITION_OFFSET + (command_position * (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) / (DXL_MAX_ANGLE - DXL_MIN_ANGLE)));
    }

    double Motor::convert_to_angle(double position)
    {
        return static_cast<double>(position - DXL_MOTOR_POSITION_OFFSET) * (DXL_MAX_ANGLE - DXL_MIN_ANGLE) / (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE);
    }

}