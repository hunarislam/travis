#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/ros.h"
#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/nav_msgs/Odometry.h"
#include "/home/foodl/esp/esp-idf/components/rosserial_esp32/include/geometry_msgs/TransformStamped.h"
#include "esp_chatter.h"
#include <string>
// #include <tf/transform_broadcaster.h>
#include <math.h>

using namespace std;

ros::NodeHandle nh;

nav_msgs::Odometry odomMsg;
// tf::TransformBroadcaster tf_broadcaster;
ros::Publisher odom_pub("odom", &odomMsg);

float last_time = 0.0;
float linear_x = 0.0;
float angular_z = 0.0;

// Odometry variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  linear_x = cmd_vel.linear.x;
  angular_z = cmd_vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

void rosserialSetup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(cmd_vel_sub);
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
  float dt = current_time.toSec() - last_time.toSec();

  // Calculate odometry values
  float delta_x = vel_x * cos(theta) * dt;
  float delta_y = vel_x * sin(theta) * dt;
  float delta_theta = vel_z * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  // Create and populate the Odometry message
  odomMsg.header.stamp = nh.now();
  odomMsg.header.frame_id = "odom";
  odomMsg.child_frame_id = "base_footprint";

  // Position
  odomMsg.pose.pose.position.x = x;
  odomMsg.pose.pose.position.y = y;
  odomMsg.pose.pose.position.z = 0.0;

  // Orientation
  odomMsg.pose.pose.orientation.x = 0.0;
  odomMsg.pose.pose.orientation.y = 0.0;
  odomMsg.pose.pose.orientation.z = sin(theta / 2.0);
  odomMsg.pose.pose.orientation.w = cos(theta / 2.0);

  // Linear velocity
  odomMsg.twist.twist.linear.x = vel_x;
  odomMsg.twist.twist.linear.y = 0.0;
  odomMsg.twist.twist.linear.z = 0.0;

  // Angular velocity
  odomMsg.twist.twist.angular.x = 0.0;
  odomMsg.twist.twist.angular.y = 0.0;
  odomMsg.twist.twist.angular.z = vel_z;

  // Publish the Odometry message
  odom_pub.publish(&odomMsg);

  // Calculate and publish the transform
  // geometry_msgs::TransformStamped odom_tf;

  // odom_tf.header.stamp = nh.now();;
  // odom_tf.header.frame_id = "odom";
  // odom_tf.child_frame_id = "base_link";
  // odom_tf.transform.translation.x = x;
  // odom_tf.transform.translation.y = y;
  // odom_tf.transform.translation.z = 0.0;
  // odom_tf.transform.rotation.x = 0.0;
  // odom_tf.transform.rotation.y = 0.0;
  // odom_tf.transform.rotation.z = sin(theta / 2.0);
  // odom_tf.transform.rotation.w = cos(theta / 2.0);


  // tf_broadcaster.sendTransform(odom_tf);

  // Update the last call time
  last_time = current_time;

  // Spin once to handle callbacks
  nh.spinOnce();
}
