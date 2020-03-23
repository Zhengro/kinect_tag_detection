#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from apriltag_ros.msg import AprilTagDetectionArray


class TagDetection:

  def __init__(self):
    
    self.tag_ids_pub = rospy.Publisher('detected_tag_ids', Int32MultiArray, queue_size=10)
    self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)


  def callback(self, data):

    rospy.loginfo(data)
    print('\n')
    ids = []
    for detection in data.detections:
        ids.append(detection.id[0])

    msg_data = Int32MultiArray()
    msg_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    msg_data.layout.dim[0].label = 'num_detection'
    msg_data.layout.dim[0].size = 1
    msg_data.layout.dim[0].stride = 1*len(ids)
    msg_data.layout.dim[1].label = 'num_ids'
    msg_data.layout.dim[1].size = len(ids)
    msg_data.layout.dim[1].stride = len(ids)
    msg_data.data = ids

    rospy.loginfo(msg_data)
    print('\n')
    self.tag_ids_pub.publish(msg_data)


if __name__ == '__main__':

    rospy.init_node('tag_ids_publisher', anonymous=True)
    td = TagDetection()
    try:
	rospy.spin()
    except KeyboardInterrupt:
        pass
