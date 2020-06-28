#ifndef XENTRINO_BASE_CONFIG_H
#define XENTRINO_BASE_CONFIG_H

#define DEBUG 1

#define K_P 0.6 // P constant
#define K_I  0.65 // 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 105                // motor's maximum RPM
#define COUNTS_PER_REV 6533        // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.127       // wheel's diameter in meters
#define PWM_BITS 8                 // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.2032  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.254   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415   // max steering angle. This only applies to Ackermann steering

// #define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

//uncomment the base you're building
#define XENTRINO_BASE DIFFERENTIAL_DRIVE            // 2WD and Tracked robot w/ 2 motors
// #define XENTRINO_BASE SKID_STEER      // 4WD robot
// #define XENTRINO_BASE ACKERMANN       // 2WD Car-like steering robot w/ 2 motors
// #define XENTRINO_BASE ACKERMANN1      // 1WD Car-like steering robot w/ 1 motor
// #define XENTRINO_BASE MECANUM         // 4WD Mecanum drive robot

//uncomment the motor driver you're using
#define TEENSY_2WD_DRIVER
//#define 4WD_TEENSY_MOTO_DRIVER
//#define 4WD_MEGA_MOTO_DRIVER
//#define USE_BTS7960_DRIVER
//#define USE_ESC

//uncomment the IMU you're using
// #define USE_GY85_IMU
 #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//TEENSY_2WD_DRIVER
#ifdef TEENSY_2WD_DRIVER
  #define MOTOR_DRIVER MOTO
  
// ENCODER 1
  #define MOTOR1_ENCODER_A  23
  #define MOTOR1_ENCODER_B  22
// ENCODER 2
  #define MOTOR2_ENCODER_A   6   // inverted
  #define MOTOR2_ENCODER_B   21
// ENCODER 3
  #define MOTOR3_ENCODER_A  24
  #define MOTOR3_ENCODER_B  24
// ENCODER 4
  #define MOTOR4_ENCODER_A  24  // inverted
  #define MOTOR4_ENCODER_B  24
  

// MOTOR 1
  #define MOTOR1_PWM  4   // D5 PWM  motor 1
  #define MOTOR1_IN_A 2   // D7 Clock wise Motor 1
  #define MOTOR1_IN_B 1   // D8 Counter clock wise Motor 1
  
// MOTOR 2
  #define MOTOR2_PWM  3   // D6 PWM motor 2
  #define MOTOR2_IN_A 5   // D4 Clock wise Motor 2
  #define MOTOR2_IN_B 0   // D9 Counter clock wise Motor 2
// MOTOR 1
  #define MOTOR3_PWM   24  // D5 PWM  motor 1
  #define MOTOR3_IN_A  24  // D7 Clock wise Motor 1
  #define MOTOR3_IN_B  24  // D8 Counter clock wise Motor 1
  
// MOTOR 2
  #define MOTOR4_PWM   24  // D6 PWM motor 2
  #define MOTOR4_IN_A  24  // D4 Clock wise Motor 2
  #define MOTOR4_IN_B   24 // D9 Counter clock wise Motor 2
    
  
// ENABLE MOTORS
  #define MOTOR1_MOTO_EN1  7    // Activalte motor 1 
  #define MOTOR2_MOTO_EN2  8    // Activalte motor 2 
  #define MOTOR3_MOTO_EN3  24   // Activalte motor 1 
  #define MOTOR4_MOTO_EN4  24   // Activalte motor 2 
  
  #define PWM_MAX pow(2, PWM_BITS) - 1  //255
  #define PWM_MIN -PWM_MAX
#endif 

//TEENSY_4WD_DRIVER
#ifdef TEENSY_4WD_DRIVER
  #define MOTOR_DRIVER MOTO1
    
// ENCODER 1
  #define MOTOR1_ENCODER_A 21
  #define MOTOR1_ENCODER_B 6 
// ENCODER 2
  #define MOTOR2_ENCODER_A 22
  #define MOTOR2_ENCODER_B 23
// ENCODER 3  
  #define MOTOR3_ENCODER_A 30
  #define MOTOR3_ENCODER_B 30 
// ENCODER 4
  #define MOTOR4_ENCODER_A 30
  #define MOTOR4_ENCODER_B 30
  
// MOTOR 1
  #define MOTOR1_PWM  21
  #define MOTOR1_IN_A 7
  #define MOTOR1_IN_B 8
// MOTOR 2
  #define MOTOR2_PWM  22
  #define MOTOR2_IN_A 20
  #define MOTOR2_IN_B 23
// MOTOR 3
  #define MOTOR3_PWM  3
  #define MOTOR3_IN_A 0
  #define MOTOR3_IN_B 1
// MOTOR 4
  #define MOTOR4_PWM  4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 5
  
// ENABLE MOTORS  
  #define MOTOR2_MOTO1_EN2 13
  #define MOTOR4_MOTO1_EN4  6 

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

// MEGA_4WD_DRIVER
#ifdef MEGA_4WD_DRIVER
  #define MOTOR_DRIVER MOTO2

/// ENCODER PINS
  #define MOTOR1_ENCODER_A 15
  #define MOTOR1_ENCODER_B  3 

  #define MOTOR2_ENCODER_A 17
  #define MOTOR2_ENCODER_B 2 

  #define MOTOR3_ENCODER_A 40
  #define MOTOR3_ENCODER_B 18 

  #define MOTOR4_ENCODER_A 42
  #define MOTOR4_ENCODER_B 19

// MOTOR 1
  #define MOTOR1_PWM  43
  #define MOTOR1_IN_A 34
  #define MOTOR1_IN_B 36
// MOTOR 2
  #define MOTOR2_PWM  44
  #define MOTOR2_IN_A 32
  #define MOTOR2_IN_B 38
// MOTOR 3
  #define MOTOR3_PWM  6
  #define MOTOR3_IN_A 7
  #define MOTOR3_IN_B 9
// MOTOR 4
  #define MOTOR4_PWM  5
  #define MOTOR4_IN_A 8
  #define MOTOR4_IN_B 11
  
  #define MOTOR1_MOTO2_EN1 A6  //activalte motor  1 
  #define MOTOR2_MOTO2_EN2 A4  //activalte motor  2 
  #define MOTOR3_MOTO2_EN3 A15  //activalte motor 3
  #define MOTOR4_MOTO2_EN4 A13  //activalte motor 4 
  
  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#endif