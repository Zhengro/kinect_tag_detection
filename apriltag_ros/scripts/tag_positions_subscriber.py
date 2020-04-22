#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray


def callback(data):

    rospy.loginfo(data)
    print('\n')


def listener():
    """
    A subscriber that subscribes to the topic detected_tag_positions.
    """
    rospy.init_node('tag_positions_subscriber', anonymous=True)
    rospy.Subscriber('detected_tag_positions', Float64MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
