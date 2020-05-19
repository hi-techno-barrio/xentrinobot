

/*
Christopher Coballes
ROS-Philippines
Hi-Techno Barrio

Project: XentrinoBot
Funded by: TAPI-DOST

Special Thanks:
Juan Jimeno Linorobot Developer
ROS OpenSource Robotics
PSCoE-Computer Engineering Society.

*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <stdio.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include "robot_specs.h"
#include "Encoder.h"


//Motor Shield headers
#include <Wire.h>
#define sign(x) (x > 0) - (x < 0)
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
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
  
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

//MOTOR 1
#define MOTOR_A1_PIN 2
#define MOTOR_B1_PIN 1 
#define PWM_MOTOR_1 4 
//MOTOR 2
#define MOTOR_A2_PIN  5
#define MOTOR_B2_PIN  0
#define PWM_MOTOR_2 3 


// #define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
struct robotFrame
{

 int max_rpm;
 float wheels_x_distance;
 float wheels_y_distance;
 float pwm_res;
 float wheel_circumference;
 int total_wheels;
};


 
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
  

struct PID 
{
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
};


float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

Encoder encoder1(2, 17);
Encoder encoder2(3, 15);

void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;
//ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", twist_to_cmd_RPM);

geometry_msgs::Vector3Stamped real_vel_msg;
//ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher rpm_pub("rpm", &real_vel_msg);

ros::Time current_time;
ros::Time last_time;

void setup() {

 int_Motor(1);
 int_Motor(2);
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);

}

void loop() {

runROS();
printDebug( 1 );
  
} // loop


/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void runROS()
{
static unsigned long prev_control_time = 0;
//this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

 //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
         prev_control_time = millis();
    }  
}


/*
void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}
*/


/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
rpm  IF_kinematics_RPM( float   linear_Vx, float linear_Vy , float angular_Vz)
{
  int rpm_req1;
  int rpm_req2;
  struct rpm rpm;
 // int  max_rpm_ ;
 
if (angular_Vz == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = linear_Vx*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (linear_Vx == 0) {
    // convert rad/s to rpm
    rpm_req2 = angular_Vz*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = - rpm_req2;
  }
  else {
    rpm_req1 = linear_Vx*60/(pi*wheel_diameter) - angular_Vz*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = linear_Vx*60/(pi*wheel_diameter) + angular_Vz*track_width*60/(wheel_diameter*pi*2);
  }
   rpm.motor1 = constrain(rpm_req1, -MAX_RPM, MAX_RPM);
   rpm.motor2 = constrain(rpm_req2, -MAX_RPM, MAX_RPM);
  // return  rpm ;
}

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg) {
  double g_req_linear_vel_x  = cmd_msg.linear.x;
  double g_req_linear_vel_y = cmd_msg.angular.z;
  double g_req_angular_vel_z  = cmd_msg.angular.y;  // zero
 g_prev_command_time = millis();
}

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
velocities cmd_to_twist_VEL( int rpm1,  int rpm2)
 {
   struct robotFrame frame;
   struct velocities vel;
   //struct vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((float)(rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * frame.wheel_circumference ; // m/s
    
    average_rps_y = ((float)(-rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 ) / frame.total_wheels) / 60;
    vel.angular_z =  (average_rps_a * frame.wheel_circumference) / ((frame.wheels_x_distance / 2) + (frame.wheels_y_distance / 2)); //  rad/s
    return vel;
}

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/

int computePid( double targetValue, double currentValue) 
{
  struct PID pid ;
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_PWM = 0;

  static double last_Error = 0;
  static double int_Error = 0;

  error = targetValue-currentValue;
  int_Error += error;
    
  pidTerm = pid.Kp*error + pid.Ki*int_Error + pid.Kd*(error-last_Error) ;
  last_Error = error;
  
  new_PWM = constrain( pidTerm, -PWM_MAX, PWM_MAX);
  
  return new_PWM;
}

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
int get_current_RPM(   long  current_encoder_ticks, int counts_per_rev )
{
    static  unsigned long prev_encoder_time = 0;
    static double prev_encoder_ticks = 0 ;
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_encoder_time = millis();
    unsigned long dt = current_encoder_time - prev_encoder_time;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    double delta_ticks = current_encoder_ticks - prev_encoder_ticks;

    //calculate wheel's speed (RPM)
    prev_encoder_time = current_encoder_time;
    prev_encoder_ticks =  current_encoder_ticks;
    
    return (delta_ticks / counts_per_rev) / dtm;
  }

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/  
void  int_Motor ( int int_motor )
{
    int   motor_pinA  ;
    int   motor_pinB ;
    int   pwm_pin ;
      
    switch (int_motor)
    {
      case 1:
       motor_pinA = MOTOR_A1_PIN ;
       motor_pinB  = MOTOR_B1_PIN ;
       pwm_pin = PWM_MOTOR_1;
      break;

      case 2: 
       motor_pinA = MOTOR_A1_PIN ;
       motor_pinB  = MOTOR_B1_PIN ;
       pwm_pin = PWM_MOTOR_1;
      break;
    }
    
    pinMode(pwm_pin, OUTPUT);
    pinMode(motor_pinA, OUTPUT);
    pinMode(motor_pinB, OUTPUT);

    //ensure that the motor is in neutral state during bootup
   analogWrite(pwm_pin, abs(0));
}

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void motorSpin ( int int_motor,int pwm)
{
    int   motor_pinA  ;
    int   motor_pinB ;
    int   pwm_pin ;
      
    switch (int_motor)
    {
      case 1:
       motor_pinA = MOTOR_A1_PIN ;
       motor_pinB  = MOTOR_B1_PIN ;
       pwm_pin = PWM_MOTOR_1;
      break;

      case 2: 
       motor_pinA = MOTOR_A1_PIN ;
       motor_pinB  = MOTOR_B1_PIN ;
       pwm_pin = PWM_MOTOR_1;
      break;
    }
            if(pwm > 0)
            {
                digitalWrite(motor_pinA, HIGH);
                digitalWrite(motor_pinB, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA, LOW);
                digitalWrite(motor_pinB, HIGH);
            }
            analogWrite(pwm_pin, abs(pwm));

}// motor spin

/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void moveBase()
{

 IF_kinematics_RPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    struct pwm pwm;
    struct rpm req_rpm;
    int current_rpm1 =  get_current_RPM (encoder1.read(),400);
    int current_rpm2 =  get_current_RPM (encoder2.read(),400);

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    /* req_rpm.motor from the cmd_twist_RPM */
    pwm.motor1 = computePid(req_rpm.motor1, current_rpm1);
    pwm.motor2 = computePid(req_rpm.motor2, current_rpm2);

   motorSpin(1,pwm.motor1);
   motorSpin(2,pwm.motor2);
   
   velocities current_vel;
   current_vel = cmd_to_twist_VEL(current_rpm1, current_rpm2);
    
    //pass velocities to publisher object
   // real_vel_msg.x = current_vel.linear_x;
  //  real_vel_msg.y = current_vel.linear_y;
   // real_vel_msg.z = current_vel.angular_z;

    //publish raw_vel_msg
   rpm_pub.publish(&real_vel_msg);
}
/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}
/*------------------------------------------------------------------------
 * 
 * 
 -------------------------------------------------------------------------*/
void printDebug(boolean DEBUG)
{
  static unsigned long prev_debug_time = 0; 
    char buffer[50];
    if(DEBUG)
       {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
              sprintf (buffer, "Encoder FrontLeft  : %ld", encoder1.read());
               nh.loginfo(buffer);
               sprintf (buffer, "Encoder FrontRight : %ld", encoder2.read());
               nh.loginfo(buffer);
               
            prev_debug_time = millis();
        }
     }  
   
}
