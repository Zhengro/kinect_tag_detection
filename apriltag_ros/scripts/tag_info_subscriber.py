#!/usr/bin/env python

import rospy
from apriltag_ros.msg import TagInfo


def callback(data):

    rospy.loginfo(data)
    print('\n')


def listener():
    """
    A subscriber that subscribes to the topic tag_info.
    """
    rospy.init_node('tag_info_subscriber', anonymous=True)
    rospy.Subscriber('tag_info', TagInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
