#!/usr/bin/env python

import sys
import rospy
import os
import time
import numpy as np
import math

import tf
from geometry_msgs.msg import PoseStamped

def tf_broadcaster_callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     np.array([msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]),
                     rospy.Time.now(),
                     'krock',
                     "world")

    angle = 0 # 1.57
    br.sendTransform((0.16, 0, 0),
                     np.array([math.sin(angle/2) * 0,
                               math.sin(angle/2) * 1,
                               math.sin(angle/2) * 0,
                               math.cos(angle/2)]),
                     rospy.Time.now(),
                     'camera',
                     "krock")


if __name__ == '__main__':
    rospy.init_node('krock_pseudo_tf_broadcaster')
    rospy.Subscriber('/krock/pose',
                     PoseStamped,
                     tf_broadcaster_callback)
    rospy.spin()
