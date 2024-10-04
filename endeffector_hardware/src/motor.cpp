#include <cmath>
#include <string>
#include <algorithm>
#include <iostream>
#include <math.h>

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
// #define DXL_MAX_ANGLE 300
// #define DXL_MIN_ANGLE 0
#define LOGGER_IDENTIFIER "endeffector_hardware > motor"

const double SPEED_CONVERSION_FACTOR = MAX_MOVING_SPEED_VALUE / (MAX_REV_PER_MIN * 2 * M_PI / 60.0);
const double DXL_RAD_MAX = 5.236;
const double DXL_RAD_MIDDLE_MAX = 2.618;

const double DXL_RAD_MIN = 0.0;
const double DXL_RPM_UNIT = 0.111;
const double RPM_TO_RAD_PER_SEC = 2 * M_PI / 60.0;
const double RAD_PER_SEC_TO_RPM = 60.0 / (2 * M_PI);

namespace endeffector_hardware
{
    void Motor::init(const std::string name, const uint8_t motor_id)
    {
        motor_name = name;
        motor_id_ = motor_id;

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "JOINT: %s ", name.c_str());
    }

    bool Motor::setup(const Comms &connection)
    {
        connection_ = connection;

        bool is_success = true;

        // Enable Torque of DYNAMIXEL
        is_success = connection_.write1ByteTxRx(motor_id_, ADDR_TORQUE_ENABLE, 1);

        // RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Zeroing....");

        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to enable torque.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to enable torque.");

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

    // takes in radians
    // does not have bounding set up
    bool Motor::write_position(double desired_angle)
    {

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "current_pos: %f, new position: %f", current_motor_angle, desired_angle);

        if ((current_motor_angle != desired_angle) && (is_moving() == false))
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "writing new position: %f", desired_angle);
            bool is_success = connection_.write2ByteTxRx(motor_id_, ADDR_GOAL_POSITION, rad_2_byte(desired_angle));
            if (!is_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set joint position.");
                return false;
            }
        }

        return true;
    }
    // radains per sec
    bool Motor::write_speed(double desired_velocity)
    {

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "current_velocity: %f, new velocity: %f", current_motor_speed, desired_velocity);

        if (current_motor_angle != desired_velocity)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "writing new position: %f", desired_motor_speed);
            bool is_success = connection_.write2ByteTxRx(motor_id_, ADDR_MOVING_SPEED, rad_2_motor_speed(desired_velocity));
            if (!is_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set joint position.");
                return false;
            }
        }

        return true;
    }

    bool Motor::update_velocity()
    {

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Reading Motor velocity for id %d", motor_id_);
        bool is_success = connection_.read2ByteTxRx(motor_id_, ADDR_PRESENT_SPEED, &current_motor_vel_);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read Wheel rotation.");
            return false;
        }
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Updating Velocity to %d", current_motor_vel_);

        current_motor_speed = byte_2_rpm(current_motor_vel_);

        return true;
    }

    bool Motor::update_position()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Reading Motor Position for id %d: ", motor_id_);
        bool is_success = connection_.read2ByteTxRx(motor_id_, ADDR_PRESENT_POSITION, &current_motor_pos_);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read angle for id %d: ", motor_id_);
            return false;
        }
        current_motor_angle = byte_2_rad(current_motor_pos_);
        return true;
    }

    bool Motor::is_moving()
    {

        bool is_success = connection_.read1ByteTxRx(motor_id_, ADDR_MOVING, &moving_);
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Is moving %d", moving_);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read angle for id %d: ", motor_id_);
            return false;
        }
        if (moving_ == 1)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "in if statement for Is moving %d", moving_);
            return true;
        }
        return false;
    }

    //  Accounting for offset
    uint16_t Motor::rad_2_byte(double radians)
    {

        // check to see if radians are within the valid range [-2.618, 2.618]
        if (radians < -DXL_RAD_MIDDLE_MAX || radians > DXL_RAD_MIDDLE_MAX)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Radians must be in the range [-2.618, 2.618]");
        }
        // return (scaledValue + DXL_MOTOR_POSITION_OFFSET) % DXL_MAX_POSITION_VALUE;
        return static_cast<uint16_t>(round((radians / DXL_RAD_MIDDLE_MAX) * DXL_MOTOR_POSITION_OFFSET + DXL_MOTOR_POSITION_OFFSET));
    }

    // Accounting for offset
    double Motor::byte_2_rad(uint16_t byte)
    {

        if (byte < 0 || byte > DXL_MAX_POSITION_VALUE)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Position must be in the range [0, 1023] for id %d: ", motor_id_);
            // throw std::out_of_range("Position must be in the range [0, 1023]");
        }
        return (static_cast<double>(byte) - DXL_MOTOR_POSITION_OFFSET) * (DXL_RAD_MIDDLE_MAX / DXL_MOTOR_POSITION_OFFSET);
    }

    double Motor::rad_2_deg(double radians)
    {
        return radians * (180.0 / M_PI);
    }
    // actually gives radians per sec
    double Motor::byte_2_rpm(uint16_t byte)
    {
        if (byte < 0 || byte > 2047)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Value must be in the range [0, 2047]");
        }

        double rpm;
        double rad_per_sec;
        if (byte >= 0 && byte <= 1023)
        {
            // Calculate CCW speed (RPM)
            rpm = byte * DXL_RPM_UNIT;
            rad_per_sec = rpm * RPM_TO_RAD_PER_SEC;
        }
        else if (byte >= 1024 && byte <= 2047)
        {
            // Calculate CW speed (RPM)
            rpm = (byte - 1024) * DXL_RPM_UNIT;
            rad_per_sec = rpm * RPM_TO_RAD_PER_SEC;
        }
        return rad_per_sec;
    }
    uint16_t Motor::rad_2_motor_speed(double rad_per_sec)
    {
        if (rad_per_sec < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Radians per second must be non-negative for this range (0-1024)");
        }

        // Convert radians per second to RPM
        double rpm = rad_per_sec * RAD_PER_SEC_TO_RPM;

        // Convert RPM to motor speed value (in the range 0-1024)
        int value = static_cast<int>(round(rpm / DXL_RPM_UNIT));

        // Ensure the motor speed value does not exceed 1024
        if (value > 1024)
        {
            value = 1024;
        }

        return value;
    }
}