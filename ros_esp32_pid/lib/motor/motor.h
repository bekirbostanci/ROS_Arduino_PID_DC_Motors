#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class controller
{
    public:
        enum driver {BDC15A,L298, BTS7960};
        controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        void spin(int pwm);

    private:
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
};

#endif