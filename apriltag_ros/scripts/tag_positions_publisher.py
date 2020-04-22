#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from apriltag_ros.msg import AprilTagDetectionArray


class TagDetection(object):
    """
    A combination of a subscriber that subscribes to the topic /tag_detections from apriltag_ros and a publisher that publishes only positions extracted from that topic.
    """

    def __init__(self):
	self.tag_pos_pub = rospy.Publisher('detected_tag_positions', Float64MultiArray, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)


    def callback(self, data):
	rospy.loginfo(data)  # /tag_detections
        print('\n')
        pos = []
        for detection in data.detections:
            pos.append(detection.pose.pose.pose.position.x)
            pos.append(detection.pose.pose.pose.position.y)
            pos.append(detection.pose.pose.pose.position.z)

        msg_data = Float64MultiArray()
        msg_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        msg_data.layout.dim[0].label = 'num_detection'
        msg_data.layout.dim[0].size = 1
        msg_data.layout.dim[0].stride = 1*len(pos)
        msg_data.layout.dim[1].label = 'num_pos'
        msg_data.layout.dim[1].size = len(pos)
        msg_data.layout.dim[1].stride = len(pos)
        msg_data.data = pos

        rospy.loginfo(msg_data)  # detected_tag_positions
        print('\n')
        self.tag_pos_pub.publish(msg_data)


if __name__ == '__main__':

    rospy.init_node('tag_positions_publisher', anonymous=True)
    td = TagDetection()
    try:
	rospy.spin()
    except KeyboardInterrupt:
        pass
