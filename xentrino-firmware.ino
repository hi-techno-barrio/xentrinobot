

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

//ROS headers
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

// #define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5
/* 
typedef enum robot
{
int weel_diameter
int Lbase;
int Xbase;
}  p, y;
*/

//struct robot
//{
//base robot_base
 int max_rpm_;
 float wheels_x_distance_;
 float wheels_y_distance_;
 float pwm_res_;
 float wheel_circumference_;
 int total_wheels_;
//};
struct port
{
 int pwm_pin;
 int motor_pinA;
 int motor_pinB;
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
  
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;
unsigned long prev_debug_time  = 0;
int counts_per_rev_  = 0;
unsigned long prev_update_time_  = 0;
long prev_encoder_ticks_  = 0;


float Kp =   0.5;
float Kd =   0;
float Ki =   0;

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

 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);

}

void loop() {

static unsigned long prev_debug_time = 0;

runROS();
printDebug( 1 );
  
} // loop


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

void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg) {
  double g_req_linear_vel_x  = cmd_msg.linear.x;
  double g_req_linear_vel_y = cmd_msg.angular.z;
  double g_req_angular_vel_z  = cmd_msg.angular.y;  // zero
 g_prev_command_time = millis();
}


rpm  IF_kinematics_RPM( float   linear_Vx, float linear_Vy , float angular_Vz)
{
  int rpm_req1;
  int rpm_req2;
  struct rpm rpm;
  int  max_rpm_ ;
 
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
   rpm.motor1 = constrain(rpm_req1, -max_rpm_, max_rpm_);
   rpm.motor2 = constrain(rpm_req2, -max_rpm_, max_rpm_);
  // return  rpm ;
}

velocities cmd_to_twist_VEL( int rpm1,  int rpm2)
 {
   struct velocities vel;
   //struct vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((float)(rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_ ; // m/s
    
    average_rps_y = ((float)(-rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 ) / total_wheels_) / 60;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s
    return vel;
}



int computePid( double targetValue, double currentValue) 
{
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_PWM = 0;

  static double last_Error = 0;
  static double int_Error = 0;

  error = targetValue-currentValue;
  int_Error += error;
    
  pidTerm = Kp*error + Ki*int_Error + Kd*(error-last_Error) ;
  last_Error = error;
  
  new_PWM = constrain( pidTerm, -MAX_RPM, MAX_RPM);
  
  return new_PWM;
}


int get_current_RPM(   long encoder_ticks )
{

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

void moveBase()
{
 // kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

// rpm =IF_kinematics(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
 IF_kinematics_RPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
     struct pwm pwm;
     struct rpm req_rpm;
    int current_rpm1 =  get_current_RPM (encoder1.read());
    int current_rpm2 =  get_current_RPM (encoder2.read());

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    /* req_rpm.motor from the cmd_twist_RPM */
    pwm.motor1 = computePid(req_rpm.motor1, current_rpm1);
    pwm.motor2 = computePid(req_rpm.motor2, current_rpm2);

//    motor1_controller.spin(pwm.motor1);
 //   motor2_controller.spin(pwm.motor2);
   

     velocities current_vel;
    current_vel = cmd_to_twist_VEL(current_rpm1, current_rpm2);
    
    //pass velocities to publisher object
   // real_vel_msg.x = current_vel.linear_x;
  //  real_vel_msg.y = current_vel.linear_y;
   // real_vel_msg.z = current_vel.angular_z;

    //publish raw_vel_msg
      rpm_pub.publish(&real_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void printDebug(boolean DEBUG)
{
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

void InitMotor ()
{
  
}

port  controllerPort ( int pwm_pin_, int motor_pinA_, int motor_pinB_ )
{
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
}

/*
void Motor (int pwm)
{
    
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

    }
