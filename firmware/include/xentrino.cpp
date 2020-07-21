#include "Arduino.h"
#include "xentrino_base_config.h"
#include "xentrino.h"

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/

Kinematics::Kinematics(  int max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance) 
  {   
      wheel_circumference = PI* wheel_diameter;
	  robot_base = XENTRINO_BASE;	 
  }
	Kinematics::rpm Kinematics::expected_RPM(float linear_x, float linear_y, float angular_z)
		{			
			float tangential_vel;
			float x_rpm;
			float y_rpm;
			float tan_rpm;	
			
			   switch(robot_base)
                {
                 case DIFFERENTIAL_DRIVE:    
				      total_wheels = 2;
					  linear_y=0.0;
				      break;
				 case SKID_STEER:           
				      total_wheels = 4;	
                      linear_y=0.0;				  
				     break;				 
                 case OMNI:  
				      total_wheels = 4;
					 // linear_y = 0.0;
					 // angular_z = 0.0;
				     break;
                 case MECANUM:               
				      total_wheels = 4;
					  //as is 
				 break;
  
                }	
			Kinematics::rpm rpm;
			// float linear_y is zero  and convert m/s to m/min			
			tangential_vel = angular_z * 60 * ((wheels_x_distance / 2) + (wheels_y_distance/ 2));
			
			x_rpm = linear_x * 60/ wheel_circumference;	
			y_rpm = linear_y * 60 / wheel_circumference;
			tan_rpm =  tangential_vel / wheel_circumference;
			
			//calculate for the target motor RPM and direction-front-left motor
			//front-left motor
			rpm.motor1 = x_rpm - y_rpm - tan_rpm;
			rpm.motor1 = constrain(rpm.motor1, -max_rpm, max_rpm);

			//front-right motor
			rpm.motor2 = x_rpm + y_rpm + tan_rpm;
			rpm.motor2 = constrain(rpm.motor2, -max_rpm, max_rpm);

			//rear-left motor
			rpm.motor3 = x_rpm + y_rpm - tan_rpm;
			rpm.motor3 = constrain(rpm.motor3, -max_rpm, max_rpm);

			//rear-right motor
			rpm.motor4 = x_rpm - y_rpm + tan_rpm;
			rpm.motor4 = constrain(rpm.motor4, -max_rpm, max_rpm);

			return rpm;			
		}


		Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2,int rpm3, int rpm4)
		{
			Kinematics::velocities vel;
			float average_rps_x;
			float average_rps_y;
			float average_rps_a;

			//convert average revolutions per minute to revolutions per second
			average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels) / 60; // RPM
			vel.linear_x = average_rps_x * wheel_circumference; // m/s

			//convert average revolutions per minute in y axis to revolutions per second
			average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels) / 60; // RPM
			if(base_platform == MECANUM)
				vel.linear_y = average_rps_y * wheel_circumference; // m/s
			else
				vel.linear_y = 0;

			//convert average revolutions per minute to revolutions per second
			average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels) / 60;
			vel.angular_z =  (average_rps_a * wheel_circumference) / ((wheels_x_distance / 2) + (wheels_y_distance / 2)); //  rad/s

    return vel;
		}
		
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Controller::Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB)  
{
	motor_driver_ = motor_driver;
    pwm_pin_      = pwm_pin ;
    motor_pinA_   = motor_pinA ;
    motor_pinB_   = motor_pinB ;
	
    switch (motor_driver)
    {
         case L298P:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
		case MOTO:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
       case MOTO1:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
       case MOTO2:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
        case BIG_MOTO:
		    enableMOTOR( motor_driver_);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);
            break;

    }
}


/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void Controller::spin(int pwm)
{
    switch (motor_driver_)
    {
		
		case L298P:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;
			
        case MOTO:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;
			
     case MOTO1:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;
			
     case MOTO2:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;			
        case BIG_MOTO:
            if (pwm > 0)
            {
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
            }

            break;
        
   
    }
}
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void  Controller::testMotor(int pwm)
{
	 analogWrite(pwm_pin_, abs(pwm));
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void Controller::enableMOTOR(driver motor_driver)
{
    switch(motor_driver)
    {
        case MOTO:   // Teensy 2WD
		pinMode(MOTOR1_MOTO_EN1,OUTPUT);
		pinMode(MOTOR2_MOTO_EN2,OUTPUT);
		digitalWrite(MOTOR1_MOTO_EN1, HIGH);
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
		break;
		
		case MOTO1:  // Teensy 4WD
		pinMode(MOTOR2_MOTO_EN2,OUTPUT);
		pinMode(MOTOR4_MOTO_EN4,OUTPUT);
		
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
        digitalWrite(MOTOR4_MOTO_EN4, HIGH); 
        break;
		
	   case MOTO2:  // MEGA  4WD
		pinMode(MOTOR1_MOTO_EN1,OUTPUT);
		pinMode(MOTOR2_MOTO_EN2,OUTPUT);
		pinMode(MOTOR3_MOTO_EN3,OUTPUT);
		pinMode(MOTOR4_MOTO_EN4,OUTPUT);
		
		digitalWrite(MOTOR1_MOTO_EN1, HIGH);
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
		digitalWrite(MOTOR3_MOTO_EN3, HIGH);
        digitalWrite(MOTOR4_MOTO_EN4, HIGH); 
        break;
		
	   case BIG_MOTO:             
		break;                  
    }
	
}  
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
PID::PID(float min_val, float max_val, float kp, float ki, float kd)
 
{
	min_val_ = min_val;
    max_val_ = max_val;
    kp_      = kp ;
    ki_      = ki ;
    kd_      = kd ;
}

double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Velocity::Velocity(int counts_per_rev )
{
	//ticks_ = ticks ;
	counts_per_rev_ = counts_per_rev ;
}
int Velocity::getRPM (int ticks)
{
		long encoder_ticks = ticks ;
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = millis();
		unsigned long dt = current_time - prev_update_time_;

		//convert the time from milliseconds to minutes
		double dtm = (double)dt / 60000;
		double delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)

		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;
		
		return (delta_ticks / counts_per_rev_) / dtm;		
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Kalman::Kalman(float N_angle,float N_bias, float N_measure, float setAngle,float setBias) {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = N_angle;    //   0.001f;
    Q_bias = N_bias;     //   0.003f;
    R_measure = N_measure;  //   0.03f;
    
    angle = setAngle ;  //   0.0f; // Reset the angle
    bias = setBias ;    //   0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
/* These are used to tune the Kalman filter */
void Kalman::setFilterNoises( float Q_angle, float Q_bias, float R_measure)
{
	Kalman::tuner tuner1; 
tuner1.Q_angle = Q_angle;
tuner1.Q_bias = Q_bias;
tuner1.R_measure = R_measure ;
}

void Kalman::getFilterNoises()
{
	Kalman::tuner tuner1; 
Q_angle = tuner1.Q_angle;
Q_bias = tuner1.Q_bias ;
R_measure = tuner1.R_measure;
}

/*	
	Link:
	http://www.daslhub.org/unlv/wiki/doku.php?id=robotino_vision_pickup
	https://javatea.adiary.jp/060
	https://programmersought.com/article/77261184260/;jsessionid=3949FE3596DA4332D91C082FEBB05B05
	http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
*/
