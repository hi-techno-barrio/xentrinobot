#ifndef XENTRINO_H
#define XENTRINO_H
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
        enum driver {L298P,MOTO2,MOTO,MOTO1, BIG_MOTO};
        Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        void spin(int pwm);
		void testMotor(int pwm);
		void enableMOTOR(driver motor_driver);

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


class Velocity 
{
public:
    Velocity(int counts_per_rev );
	int getRPM (int ticks);
	
private:
	int  ticks_ ;
	int counts_per_rev_;
	unsigned long prev_update_time_;
    long prev_encoder_ticks_;
};


class Kalman {

public:
     Kalman(float N_angle,float N_bias, float N_measure, float setAngle,float setBias);
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);	
    struct tuner
        {
           float Q_angle; // Process noise variance for the accelerometer
           float Q_bias; // Process noise variance for the gyro bias
           float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
        };

    /* These are used to tune the Kalman filter */
	/**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setFilterNoises( float Q_angle, float Q_bias, float R_measure);
	void getFilterNoises( );
    	
private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

#endif
