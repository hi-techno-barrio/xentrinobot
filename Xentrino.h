#ifndef XENTRINO_H
#define XENTRINO_H

#include "Arduino.h"

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


class Kinematics
{
    public:
        struct rpm
        {
            int motor1;
            int motor2;
           // int motor3;
           // int motor4;
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
           // int motor3;
           // int motor4;
        };

        Kinematics(int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance);    
        velocities getVelocities(int rpm1, int rpm2);
        rpm expectedRPM(float linear_x, float linear_y, float angular_z);
	
		private:
        int max_rpm;
        float wheels_x_distance;
        float wheels_y_distance;
        //float pwm_res_;
        float wheel_circumference;
        int total_wheels;

};


class Controller
{
    public:
        enum driver {MOTO, BIG_MOTO};
        Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        void spin(int pwm);

   private:
      private:       
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
       
      
       
};

class PID
{
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double compute(float setpoint, float measured_value);
        void updateConstants(float kp, float ki, float kd);

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double integral_;
        double derivative_;
        double prev_error_;
};

/*
class Decoder
{
public:
	Decoder(uint8_t pin1, uint8_t pin2, int counts_per_rev);
	int getRPM();
	
	private:
	int counts_per_rev_;
	unsigned long prev_update_time_;
    long prev_encoder_ticks_;
	int pin1_;
	int pin2_;
	//Encoder_internal_state_t encoder;
};
*/
#endif
