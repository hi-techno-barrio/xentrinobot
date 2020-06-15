#ifndef XENTRINO_BASE_CONFIG_H
#define XENTRINO_BASE_CONFIG_H

#define RPM_TO_RPS 1/60
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant
#define PI               3.1415926
#define TWO_PI           6.2831853

//define your robot' specs here
#define MAX_RPM 330               // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN


//uncomment the base you're building
#define XENTRINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define XENTRINO_BASE SKID_STEER      // 4WD robot
// #define XENTRINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define XENTRINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define XENTRINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
#define 2WD_TEENSY_MOTO_DRIVER
//#define 4WD_TEENSY_MOTO_DRIVER
//#define 4WD_MEGA_MOTO_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
// #define USE_GY85_IMU
 #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 330               // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering


/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//MOTOR PINS
#ifdef 2WD_TEENSY_MOTO_DRIVER
  #define MOTOR_DRIVER 2WD_TEENSY_MOTO

  /// ENCODER PINS
  #define MOTOR1_ENCODER_A 6
  #define MOTOR1_ENCODER_B 21 

  #define MOTOR2_ENCODER_A 22
  #define MOTOR2_ENCODER_B 23 

/// MOTOR PINS
  #define MOTOR1_PWM  4
  #define MOTOR1_IN_A 2
  #define MOTOR1_IN_B 1

  #define MOTOR2_PWM  3
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 0

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

//MOTOR PINS
#ifdef 4WD_TEENSY_MOTO_DRIVER
  #define MOTOR_DRIVER 4WD_TEENSY_MOTO
  
/// MOTOR PINS
  #define MOTOR1_PWM  21
  #define MOTOR1_IN_A 7
  #define MOTOR1_IN_B 8

  #define MOTOR2_PWM  22
  #define MOTOR2_IN_A 20
  #define MOTOR2_IN_B 23

  #define MOTOR3_PWM 3
  #define MOTOR3_IN_A 0
  #define MOTOR3_IN_B 1

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 5

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef 4WD_MEGA_MOTO_DRIVER
  #define MOTOR_DRIVER 4WD_MEGA_MOTO

/// ENCODER PINS
  #define MOTOR1_ENCODER_A 15
  #define MOTOR1_ENCODER_B  3 

  #define MOTOR2_ENCODER_A 17
  #define MOTOR2_ENCODER_B 2 

  #define MOTOR3_ENCODER_A 40
  #define MOTOR3_ENCODER_B 18 

  #define MOTOR4_ENCODER_A 42
  #define MOTOR4_ENCODER_B 19

/// MOTOR PINS
  #define MOTOR1_PWM  43
  #define MOTOR1_IN_A 34
  #define MOTOR1_IN_B 36

  #define MOTOR2_PWM  44
  #define MOTOR2_IN_A 32
  #define MOTOR2_IN_B 38

  #define MOTOR3_PWM   6
  #define MOTOR3_IN_A  7
  #define MOTOR3_IN_B  9

  #define MOTOR4_PWM   5
  #define MOTOR4_IN_A  8
  #define MOTOR4_IN_B 11

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 
// #define STEERING_PIN 7

#endif
