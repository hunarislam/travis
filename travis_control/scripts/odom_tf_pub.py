#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def odometry_callback(msg):
    # Extract relevant information from the odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Create a transform object
    transform = tf.TransformBroadcaster()

    # Set the translation
    translation = (position.x, position.y, position.z)

    # Set the rotation
    rotation = (orientation.x, orientation.y, orientation.z, orientation.w)

    # Publish the transform
    transform.sendTransform(translation, rotation, rospy.Time.now(), "base_footprint", "odom")

if __name__ == '__main__':
    rospy.init_node('odom_tf_pub')

    # Subscribe to the odometry topic
    rospy.Subscriber('odom', Odometry, odometry_callback)

    rospy.spin()

