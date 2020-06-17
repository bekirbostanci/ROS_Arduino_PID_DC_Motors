#ifndef kinematics_H
#define kinematics_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE};

        base base_platform;

        struct rpm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };

        kinematics(base robot_base, int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance);
        velocities getVelocities(int rpm1, int rpm2, int rpm3, int rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);

        int max_rpm_;
        float wheels_x_distance_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
};

#endif