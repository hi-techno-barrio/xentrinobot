#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

void commandCallback(const geometry_msgs::Twist& cmd_msg);

/* void handle_rpm( const geometry_msgs::Vector3Stamped& rpm) {
  rpm_act1 = rpm.vector.x;
  rpm_act2 = rpm.vector.y;
  rpm_dt = rpm.vector.z;
  rpm_time = rpm.header.stamp;
}  */

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;
  //  g_prev_command_time = millis();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
  
    tf2_ros::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(50.0);

    while(n.ok()){

        ros::spinOnce();
        current_time = ros::Time::now();

        double dt = (current_time-last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // Setting the transform stamped messages:
        tf2::Quaternion q;
        q.setRPY(0,0,th);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        // Send the transform:
        odom_broadcaster.sendTransform(odom_trans);

        // Send the odometry messages to ROS:
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        // Here, take the positions from the wheelme backend whenever it reaches and send it to ros:
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.x = q.y();
        odom.pose.pose.orientation.x = q.z();
        odom.pose.pose.orientation.x = q.w();


        // This values will be always zero until wheelme will start publishing its velocities:
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z  = vth;

        // Now publish all that to odom topic:
        odom_pub.publish(odom);
        last_time = current_time;
        r.sleep();

    }

}
