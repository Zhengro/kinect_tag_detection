#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray


def callback(data):
    rospy.loginfo(data)
    print('\n')


def listener():
    rospy.init_node('tag_detections_subscriber', anonymous=True)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
