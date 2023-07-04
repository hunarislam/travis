#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def odometry_callback(msg):
    # Extract relevant information from the odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Create a transform object
    transform = tf.TransformerROS()

    # Set the translation
    translation = (position.x, position.y, position.z)
    transform.setTranslation(translation)

    # Set the rotation
    rotation = (orientation.x, orientation.y, orientation.z, orientation.w)
    transform.setRotation(rotation)

    # Publish the transform
    br = tf.TransformBroadcaster()
    br.sendTransform(translation, rotation, rospy.Time.now(), "base_footprint", "odom")

if __name__ == '__main__':
    rospy.init_node('odom_tf_pub')

    # Subscribe to the odometry topic
    rospy.Subscriber('odom', Odometry, odometry_callback)

    rospy.spin()
