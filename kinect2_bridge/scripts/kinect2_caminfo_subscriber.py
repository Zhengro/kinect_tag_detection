#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo


def callback(data):

    rospy.loginfo(data)
    print('\n')


def listener():

    rospy.init_node('kinect2_listener', anonymous=True)
    rospy.Subscriber('/kinect2/qhd/camera_info', CameraInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
