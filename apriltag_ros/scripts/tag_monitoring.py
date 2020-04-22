#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from apriltag_ros.msg import AprilTagDetectionArray


class TagDetection(object):
    """
    A combination of a subscriber that subscribes to the topic /tag_detections from apriltag_ros and a publisher that publishes the position of the tag with target id.
    """

    def __init__(self):

        self.tag_pos_pub = rospy.Publisher('target_tag_position', Float64MultiArray, queue_size=10)
        self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)


    def callback(self, data):

	rospy.loginfo(data)  # /tag_detections
        print('\n')
	
	if len(data.detections) != 0:
	    detections = {}
            for detection in data.detections:
	        tag_id = detection.id[0]
	        detections['{}'.format(tag_id)] = []
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.x)
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.y)
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.z)
	    print(detections)
	    print('\n')
	
	    global target_id
	    if detections.has_key('{}'.format(target_id)):
		detection = self.target_monitoring(target_id, detections)


    def target_monitoring(self, target_id, detections):

	detection = detections['{}'.format(target_id)]
	x = detection[0]
	y = detection[1]

    	global target_area

	top_left = target_area[0]
        bottom_right = target_area[1]

    	if x <= bottom_right[0] and x >= top_left[0]:
	    if y <= bottom_right[1] and y >= top_left[1]:
	    	print('The tag {} has entered the target area!\n'.format(target_id))

		msg_data = Float64MultiArray()
		msg_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		msg_data.layout.dim[0].label = 'num_detection'
		msg_data.layout.dim[0].size = 1
		msg_data.layout.dim[0].stride = 3
		msg_data.layout.dim[1].label = 'num_para'  # x, y, z
		msg_data.layout.dim[1].size = 3
		msg_data.layout.dim[1].stride = 3
		msg_data.data = detection

		rospy.loginfo(msg_data)  # target_tag_position
		print('\n')
		self.tag_pos_pub.publish(msg_data)

	return detection


if __name__ == '__main__':

    # set the target area range by giving top left and bottom right coordinates
    target_area = [[0, -0.5], [0.5, 0]]
    # set target tag id
    target_id = 4

    rospy.init_node('tag_monitor', anonymous=True)
    td = TagDetection()
    try:
	rospy.spin()
    except KeyboardInterrupt:
        pass

