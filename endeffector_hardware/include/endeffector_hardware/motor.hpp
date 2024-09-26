#ifndef ENDEFFECTOR_HARDWARE__MOTOR_HPP_
#define ENDEFFECTOR_HARDWARE__MOTOR_HPP_

#include <string>
#include <endeffector_hardware/comms.hpp>

namespace endeffector_hardware
{
    class Motor
    {
    public:
        std::string name;

        // storing angles
        double cmd = 0;
        double pos = 0;
        double vel = 0;

        void init(const std::string name, const uint8_t motor_id);
        bool setup(const Comms &connection);
        void shutdown();

        bool write_velocity(double velocity);
        bool write_position(double position);
        bool update_position();
        double get_velocity();

    private:
        endeffector_hardware::Comms connection_;

        uint8_t motor_id_;
        // int8_t inverted_;

        uint16_t convert_to_motor_vel(double rad_vel);
        uint16_t convert_to_motor_pos(double command_position);

        double convert_to_real_vel(uint16_t dynamixel_vel);
        double convert_to_angle(double position);
    };
}

#endif