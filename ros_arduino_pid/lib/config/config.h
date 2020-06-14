#ifndef CONFIG_H
#define CONFIG_H


#define BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors

#define DEBUG 1
#define DEBUG_RATE 5
#define LOOPTIME 100		 //Looptime in millisecond


#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant


//define your robot' specs here
#define MAX_RPM             330    
#define MAX_SPEED           0.4        // motor's maximum RPM
#define COUNTS_PER_REV      1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER      0.15       // wheel's diameter in meters
#define WHEEL_BASE          0.325
#define PWM_BITS            8          // PWM Resolution of the microcontroller
#define ENCODER_PULSE       480

#define MOTOR_DRIVER BDC15A
// MOTOR DRIVERS
#ifdef MOTOR_DRIVER
#if MOTOR_DRIVER == BDC15A
    /// ENCODER PINS
    #define MOTOR1_ENCODER_A    2
    #define MOTOR1_ENCODER_B    4 

    #define MOTOR2_ENCODER_A    3
    #define MOTOR2_ENCODER_B    18 


    #define MOTOR1_PWM 6
    #define MOTOR1_IN_A 24
    #define MOTOR1_IN_B 25

    #define MOTOR2_PWM 7
    #define MOTOR2_IN_A 30
    #define MOTOR2_IN_B 31
    
    #define PWM_MAX 250
    #define PWM_MIN -250
#endif
#endif
#endif