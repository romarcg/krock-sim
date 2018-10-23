#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from webots_ros.srv import *
from webots_ros.msg import Float64ArrayStamped

def camera_cb(data):
    rospy.loginfo('camera data received.')

if __name__ == "__main__":
    FRONTAL_CAMERA = '/krock/front_camera/image'
    MANUAL_CONTROL = '/krock/manual_control_input'

    rospy.init_node("test_camera")

    mover = rospy.Publisher(MANUAL_CONTROL, Float64ArrayStamped, queue_size=1, latch = True)

    try:
        request_enable = rospy.ServiceProxy('/krock/front_camera/enable', set_int)
        res = request_enable(1)
        rospy.loginfo(res)
    except rospy.ServiceException as e:
        rospy.logerr(e)

    #_sub = rospy.Subscriber(FRONTAL_CAMERA, Image, camera_cb)

    nap = rospy.Rate(10)

    #while not rospy.is_shutdown():
    data = Float64ArrayStamped(data=[0, 1, 0.3, 0])
    mover.publish(data)
    #nap.sleep()

    while not rospy.is_shutdown():
        #mover.publish(data)
        True
