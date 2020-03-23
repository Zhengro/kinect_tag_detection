#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray


def callback(data):

    rospy.loginfo(data)
    print('\n')
    # print(data.data)  # <type 'tuple'>


def listener():
    """
    A subscriber that subscribes to the topic detected_tag_ids.
    """
    rospy.init_node('tag_ids_subscriber', anonymous=True)
    rospy.Subscriber('detected_tag_ids', Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
