#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "esp_chatter.h"
#include <string>

using namespace std;

ros::NodeHandle nh;

std_msgs::String str_msg;
std_msgs::Int32 int_msg;
ros::Publisher chatter("chatter", &int_msg);

void rosserial_setup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(chatter);
}

void rosserial_publish(int enc_msg)
{
  int_msg.data = enc_msg;
  // Send the message
  chatter.publish(&int_msg);
  nh.spinOnce();
}
