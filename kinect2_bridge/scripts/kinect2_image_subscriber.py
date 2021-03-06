#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
    
    bridge = CvBridge()
    try:
        kinect2_image = bridge.imgmsg_to_cv2(data, 'passthrough')
    except CvBridgeError as error:
        print(error)
    
    cv2.imshow('Kinect2_Color_Image', kinect2_image)
    cv2.waitKey(3)


def listener():

    rospy.init_node('kinect2_listener', anonymous=True)
    rospy.Subscriber('/kinect2/qhd/image_color', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
