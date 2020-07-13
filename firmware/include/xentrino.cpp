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
// Encoder_internal_state_t * Encoder::interruptArgs[];
/*
Decoder::Decoder(uint8_t pin1, uint8_t pin2, int counts_per_rev)
 
{
	 Encoder pass ;
    pass	 =  new  Encpder( pin1,pin2);
	//Encoder pass ;   = new Encoder(int  pin1,int pin2 );
    pin1_ = pin1;
	pin2_ = pin2;
    counts_per_rev_ = counts_per_rev;
	//prev_update_time_ = ;
     //long prev_encoder_ticks_ =;
}
int Decoder::getRPM(){
		long  int encoder_ticks = pass.read();
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
	
	Link:
	http://www.daslhub.org/unlv/wiki/doku.php?id=robotino_vision_pickup
	https://javatea.adiary.jp/060
	https://programmersought.com/article/77261184260/;jsessionid=3949FE3596DA4332D91C082FEBB05B05
	http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
*/
