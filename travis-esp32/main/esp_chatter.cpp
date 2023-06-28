#include "ros.h"
#include "nav_msgs/Odometry.h"
#include "esp_chatter.h"
#include <string>

using namespace std;

ros::NodeHandle nh;

nav_msgs::Odometry odom;
ros::Publisher chatter("odom", &odom);

void rosserial_setup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(chatter);
}

void rosserial_publish(int enc_msg)
{
  / Populate the Odometry message
  // Set the header
  odom.header.stamp = nh.now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // Set the pose
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  // Set the twist
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  // Send the message
  chatter.publish(&odom);
  nh.spinOnce();
}
