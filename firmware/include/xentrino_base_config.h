#ifndef XENTRINO_BASE_CONFIG_H
#define XENTRINO_BASE_CONFIG_H

#define RPM_TO_RPS 1/60
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant
//#define PI               3.1415926
#define TWO_PI           6.2831853

//define your robot' specs here
#define MAX_RPM 330               // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5
#define DIFFERENTIAL_DRIVE 2

#define MOTOR1_PWM  4
#define MOTOR1_IN_A 2
#define MOTOR1_IN_B 1

#define MOTOR2_PWM  3
#define MOTOR2_IN_A 5
#define MOTOR2_IN_B 0

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
#define MOTOR_DRIVER MOTO



#endif
