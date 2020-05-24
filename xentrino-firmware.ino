

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

#include "Encoder.h"
#include "xentrino.h"

//Motor Shield headers
#include <Wire.h>
#define sign(x) (x > 0) - (x < 0)

// #define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards

Encoder Encoder1(2, 17);
Encoder Encoder2(3, 15);

Controller MOTO1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller MOTO2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics( MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;


//void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;
//ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", twist_to_cmd_RPM);

geometry_msgs::Vector3Stamped real_vel_msg;
//ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher rpm_pub("rpm", &real_vel_msg);

ros::Time current_time;
ros::Time last_time;

void setup() {

 //int_Motor(1);
// int_Motor(2);
 nh.initNode();
 nh.getHardware()->setBaud(57600);
// nh.subscribe(sub);
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
int get_actual_RPM(   long  current_encoder_ticks, int counts_per_rev )
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

void moveBase()
{
 //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.expectedRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 =  get_actual_RPM (Encoder1.read(),MAX_RPM);
    int current_rpm2 =  get_actual_RPM (Encoder2.read(),MAX_RPM);

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    MOTO1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    MOTO2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
      

    Kinematics::velocities current_vel;

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
        
    //pass velocities to publisher object
   /* raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);  */
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
              sprintf (buffer, "Encoder FrontLeft  : %ld", Encoder1.read());
               nh.loginfo(buffer);
               sprintf (buffer, "Encoder FrontRight : %ld", Encoder2.read());
               nh.loginfo(buffer);
               
            prev_debug_time = millis();
        }
     }  
   
}
