
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

double  linear_velocity_x =  0 ;
double  linear_velocity_y =  0 ;
double  angular_velocity_z = 0 ;
double rate = 10.0;
double  last_vel_time = 0;
double  vel_dt        = 0;
double  x_pos         = 0;
double  y_pos         = 0;
double  heading       = 0;  // robot directions

void commandCallback(const geometry_msgs::Twist&  vel)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
   linear_velocity_x = vel.linear.x;
   linear_velocity_y = vel.linear.y;
   angular_velocity_z= vel.angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, " "xentrino_base_node"");

  ros::NodeHandle n; 
   // ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
  ros::Subscriber sub = n.subscribe("raw_vel", commandCallback);
  ros::Publisher odom_publisher_;
  // ros::Subscriber velocity_subscriber_;
  // tf2_ros::TransformBroadcaster odom_broadcaster_;
  tf2::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
    
  ros::Rate r(rate);
    
 while(n.ok())
    {
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();
    vel_dt_ = (current_time - last_vel_time).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z * vel_dt; //radians
    double delta_x = (linear_velocity_x * cos(heading) - linear_velocity_y * sin(heading)) * vel_dt; //m
    double delta_y = (linear_velocity_x * sin(heading) + linear_velocity_y * cos(heading)) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    // RPY then convert to quaternion
    odom_quat.setRPY(0,0,heading_);

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
        
    // geometry_msgs::TransformStamped odom_trans <--  robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
        
    //robot's heading in quaternion
    // geometry_msgs::TransformStamped odom_trans <-- tf2::Quaternion odom_quat;
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    //odom_broadcaster_.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // nav_msgs::Odometry odom <-- robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
        
    // nav_msgs::Odometry odom <--robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    // nav_msgs::Odometry odom <--linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
        
    // nav_msgs::Odometry odom <--angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
   // ros::Publisher odom_publisher_;
    odom_publisher_.publish(odom);
    r.sleep();
    }
//return 0;
}
