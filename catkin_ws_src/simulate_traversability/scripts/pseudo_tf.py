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


    # Calculations for blender frame. With respect to ROS
    # Blende's y = ROS' z,
    # Blende's z = ROS' y

    b_matrix_t = tf.transformations.translation_matrix((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    b_matrix_r = tf.transformations.quaternion_matrix([msg.pose.orientation.x,
              msg.pose.orientation.y,
              msg.pose.orientation.z,
              msg.pose.orientation.w])
    body_tr = np.matmul(b_matrix_t, b_matrix_r)

    c_matrix_t = tf.transformations.translation_matrix((0.16, 0, 0))
    c_matrix_r = tf.transformations.quaternion_matrix([0, 0, 0, 1])
    camera_tr = np.matmul(c_matrix_t, c_matrix_r)

    world_camera_tr = np.matmul (body_tr, camera_tr)
    print("ROS+++++++++", body_tr)
    print("ROS+++++++++", camera_tr)
    print("ROS+++++++++", world_camera_tr)


    b_matrix_t = tf.transformations.translation_matrix((msg.pose.position.x, msg.pose.position.z, msg.pose.position.y))
    b_matrix_r = tf.transformations.quaternion_matrix([msg.pose.orientation.x,
              msg.pose.orientation.z,
              msg.pose.orientation.y,
              msg.pose.orientation.w])
    body_tr = np.matmul(b_matrix_t, b_matrix_r)

    c_matrix_t = tf.transformations.translation_matrix((0.16, 0, 0))
    c_matrix_r = tf.transformations.quaternion_matrix([0, 0, 0, 1])
    camera_tr = np.matmul(c_matrix_t, c_matrix_r)

    world_camera_tr = np.matmul (body_tr, camera_tr)
    print("BLENDER+++++++++", body_tr)
    print("BLENDER+++++++++", camera_tr)
    print("BLENDER+++++++++", world_camera_tr)


if __name__ == '__main__':
    rospy.init_node('krock_pseudo_tf_broadcaster')
    rospy.Subscriber('/krock/pose',
                     PoseStamped,
                     tf_broadcaster_callback)
    rospy.spin()
