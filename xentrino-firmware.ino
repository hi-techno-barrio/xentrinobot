

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
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"
#include "encoder.h"

//Motor Shield headers
#include <Wire.h>
#define sign(x) (x > 0) - (x < 0)


unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;

float Kp =   0.5;
float Kd =   0;
float Ki =   0;

Encoder encoder1(2, 17);
Encoder encoder2(3, 15);

ros::NodeHandle nh;


void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg) {
  double linear_x  = cmd_msg.linear.x;
  double angular_z = cmd_msg.angular.z;
  double linear_y  = cmd_msg.angular.y;  // zero
  
if (angular_z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = linear_x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (linear_x == 0) {
    // convert rad/s to rpm
    rpm_req2 = angular_z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = - rpm_req2;
  }
  else {
    rpm_req1 = linear_x*60/(pi*wheel_diameter) - angular_z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = linear_x*60/(pi*wheel_diameter) + angular_z*track_width*60/(wheel_diameter*pi*2);
  }
   rpm.motor1 = constrain(rpm_req1, -max_rpm_, max_rpm_);
   rpm.motor2 = constrain(rpm_req2, -max_rpm_, max_rpm_);
   return rpm;
}

cmd_to_twist_VEL(int rpm1, int rpm2)
 {
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((float)(rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s
    
    average_rps_y = ((float)(-rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 ) / total_wheels_) / 60;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s
    return vel;
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", twist_to_cmd_RPM);

geometry_msgs::Vector3Stamped rpm_msg;

ros::Publisher rpm_pub("rpm", &rpm_msg);

ros::Time current_time;
ros::Time last_time;

void setup() {

 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);

}

void loop() {

runROS();
  
} // loop


runROS()
{

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


void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}
void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

int computePid( double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_Pwm = 0;

  static double last_Error = 0;
  static double int_Error = 0;

  error = targetValue-currentValue;
  int_Rrror += error;
    
  pidTerm = Kp*error + Ki*int_Error + Kd*(error-last_Error) ;
  last_Error = error;
  
  new_PWM = constrain( pidTerm, -MAX_RPM, MAX_RPM);
  
  return new_PWM;
}


int get_current_RPM(long encoder_pulse ){
    //long encoder_ticks = encoder.read();
    encoder_ticks = encoder_pulse;
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
  kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
   // int current_rpm1 = motor1_encoder.getRPM();
   // int current_rpm2 = motor2_encoder.getRPM();
    
    int current_rpm1 =  get_current_RPM (encoder1.read());
    int current_rpm2 =  get_current_RPM (encoder1.read());

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    pwm_RPM1 = motor1_pid.compute(req_rpm.motor1, current_rpm1));
    pwm_RPM2 =  motor2_pid.compute(req_rpm.motor2, current_rpm2));

    motor1_controller.spin(pwm_RPM1));
    motor2_controller.spin(pwm_RPM2));
   

    Kinematics::velocities current_vel;
    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}
