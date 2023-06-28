#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/ros.h"
#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/nav_msgs/Odometry.h"
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

void odom_pub(float vel_x, float vel_z)
{
  // Populate the Odometry message
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
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = vel_z;
  // Send the message
  chatter.publish(&odom);
  nh.spinOnce();
}
