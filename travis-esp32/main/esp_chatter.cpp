#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/ros.h"
#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/nav_msgs/Odometry.h"
#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/geometry_msgs/TransformStamped.h"
#include "esp_chatter.h"
#include <string>
#include <tf/transform_broadcaster.h>

using namespace std;

ros::NodeHandle nh;

nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

ros::Publisher odom_pub("odom", &odom);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

float last_time = 0.0;
float linear_x = 0.0;
float angular_z = 0.0;

void rosserialSetup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(cmd_vel_sub);
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  linear_x = cmd_vel.linear.x;
  angular_z = cmd_vel.angular.z;
}

void getCmdVel(float* vel_x, float* vel_z)
{
  *vel_x = linear_x;
  *vel_z = angular_z;
}

void publishOdometry(float vel_x, float vel_z)
{
  // Get the time
  ros::Time current_time = nh.now();

  // Calculate the time elapsed since the last call
  static ros::Time last_time = current_time;
  float dt = (current_time - last_time).toSec();

  // Set the header
  odom.header.stamp = nh.now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // Calculate the distance traveled
  float distance = vel_x * dt;

  // Calculate the change in orientation
  float delta_theta = vel_z * dt;

  // Update the pose
  odom.pose.pose.position.x += distance * cos(delta_theta);
  odom.pose.pose.position.y += distance * sin(delta_theta);
  odom.pose.pose.orientation.z = sin(delta_theta / 2);
  odom.pose.pose.orientation.w = cos(delta_theta / 2);

  // Update the twist
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.angular.z = vel_z;

  // Update the timestamp
  odom.header.stamp = current_time;

  // Publish the Odometry message
  odom_pub.publish(&odom);

  // Calculate and publish the transform
  odom_tf.header.stamp = odom.header.stamp;
  odom_tf.header.frame_id = odom.header.frame_id;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;

  tf_broadcaster.sendTransform(odom_tf);

  // Update the last call time
  last_time = current_time;

  // Spin once to handle callbacks
  nh.spinOnce();
}
