#ifndef XENTRINO_H
#define XENTRINO_H
#include "xentrino_base_config.h"
#include "Arduino.h"


class Kinematics
{
    public:
	 enum base {DIFFERENTIAL_DRIVE, SKID_STEER, OMNI, MECANUM};
        base base_platform;
		base robot_base;
		
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

        Kinematics(int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance);    
        velocities getVelocities(int rpm1, int rpm2,int rpm3, int rpm4);
        rpm expected_RPM(float linear_x, float linear_y, float angular_z);
	
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
		void enableMOTO(driver motor_driver);

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
