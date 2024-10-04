#ifndef ENDEFFECTOR_HARDWARE__MOTOR_HPP_
#define ENDEFFECTOR_HARDWARE__MOTOR_HPP_

#include <string>
#include <endeffector_hardware/comms.hpp>

namespace endeffector_hardware
{
    class Motor
    {
    public:
        std::string motor_name;

        // storing motor position [0-1023]
        // uint16_t current_pos{0};
        // storing motor position [0-2047]
        // uint16_t current_vel{0};
        // storing motor position [0-1023]
        // uint16_t desired_pos{0};
        // storing motor position [0-2047]
        // uint16_t desired_vel{0};
        // std::numeric_limits<double>::quiet_NaN();
        // storing motor angle [0-5.23599] this value isin radians
        double current_motor_angle{0.0};
        // storing motor speed this value is rotations per min
        double current_motor_speed{0.0};
        // storing desired_motor angle [0-5.23599] this value is in radians
        double desired_motor_angle{0.0};
        // storing desired_motor_speed [TBD] this value is in radians per sec
        double desired_motor_speed{0.0};

        // current motor position in motor terms [0-1023]
        uint16_t current_motor_pos_{0};
        // current motor position in motor terms [0-1023] && [1024-2047]
        uint16_t current_motor_vel_{0};

        // Initializes the motor with name and ID [Creates object]
        void init(const std::string name, const uint8_t motor_id);

        // Tests connection and Enables torque
        bool setup(const Comms &connection);

        // properly shuts down the program
        void shutdown();

        // Writes new position to this motor object
        // Takes in Value from the trajectory (radians)
        bool write_position(double motor_pos_angle);
        // bool write_velocity(double motor_vel);

        // Reads the new position to this motor object
        bool update_position();
        // Reads the new velocity to this motor object
        bool update_velocity();

        // Reads the new velocity to this motor object
        bool is_moving();

        bool write_speed(double speed);

    private:
        endeffector_hardware::Comms connection_;

        uint8_t motor_id_;
        uint8_t moving_{0};

        // converts radians to usable motor position
        // To DO: Make sure it is within the bounds
        uint16_t rad_2_byte(double radians);

        // converts motor_pos to angle in radains
        // To DO: Make sure it is within the bounds
        double byte_2_rad(uint16_t byte);

        // converts radians to degrees
        // To DO: Make sure it is within the bounds
        double rad_2_deg(double radians);

        // converts motor_velocity to usable motor speed in rpm
        // To DO: Make sure it is within the bounds
        double byte_2_rpm(uint16_t byte);

        uint16_t rad_2_motor_speed(double rad_per_sec);
    };
}

#endif