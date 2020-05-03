#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from apriltag_ros.msg import AprilTagDetectionArray, TagInfo


def enter_which_area(box_xy, target_areas):

    x = box_xy[0]
    y = box_xy[1]

    for k, v in target_areas.items():
	top_left = v[0]
	bottom_right = v[1]
	if x <= bottom_right[0] and x >= top_left[0]:
	    if y <= bottom_right[1] and y >= top_left[1]:
		print('Box has entered the {} target area!\n'.format(k))
		return 1
    return 0


class TagDetection(object):
    """
    A combination of a subscriber that subscribes to the topic /tag_detections from apriltag_ros and a publisher that publishes tag info:
    1. 1/0: if the tag enters the target areas (1) or not (0)
    2. 1/0: if the tag indicates the box is small (1) or not (0)
    3. 1/0: if the tag stays still (1) or not (0)
    4. the box tag's position [x, y, z]
    """

    def __init__(self, target_areas, box_ids, receipt_ids, pub_rate):
	
	rospy.init_node('tag_monitoring', anonymous=True)
        self.tag_info_pub = rospy.Publisher('tag_info', TagInfo, queue_size=50)
        self.tag_detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

	# setup
	self.target_areas = target_areas
	self.box_ids = box_ids
	self.receipt_ids = receipt_ids
	self.rate = rospy.Rate(pub_rate)

	# single box
	self.box_tag_id = None
	self.box_tag_position = [0, 0, 0]
	self.small_box = None
	self.enter_target_areas = 0
	self.stay_still = 0

	# single receipt
	self.receipt_tag_id = None
	self.receipt_tag_positin = None


    def callback(self, data):

	rospy.loginfo(data)  # /tag_detections
        print('\n')

	if len(data.detections) != 0:
	    # get all tag detections: ids and positions
	    detections = {}
            for detection in data.detections:
	        tag_id = detection.id[0]
	        detections['{}'.format(tag_id)] = []
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.x)
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.y)
	        detections['{}'.format(tag_id)].append(detection.pose.pose.pose.position.z)
	    print(detections)
	    print('\n')
	
	    # get the id of box tag and the box size
	    for tag_id in detections.keys():
		if self.box_tag_id == None and self.small_box == None:
		    for k, v in self.box_ids.items():
		        if int(tag_id) in v:
			    self.box_tag_id = tag_id
			    if k == 'small':
			        self.small_box = 1
			    else:
			        self.small_box = 0
			    break
		else:
		    break
		
	    # monitor box state
	    self.box_monitoring(detections['{}'.format(self.box_tag_id)])


    def box_monitoring(self, detection):	

	# check if the box enters either the left or the right area
	x = detection[0]
	y = detection[1] 
	self.enter_target_areas = enter_which_area([x, y], self.target_areas)

	# check if the box stays still
	move = [a - b for a, b in zip(detection, self.box_tag_position)]
	move_sum = sum(map(lambda i : abs(i), move))
	if move_sum < 0.001:
	    self.stay_still = 1

	# update the box tag position
	self.box_tag_position = detection

	# send out message
	msg_data = TagInfo()
	msg_data.enter_target_areas = self.enter_target_areas
	msg_data.small_box = self.small_box
	msg_data.stay_still = self.stay_still

	msg_data.box_tag_position = Float64MultiArray()
	msg_data.box_tag_position.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
	msg_data.box_tag_position.layout.dim[0].label = 'num_detection'
	msg_data.box_tag_position.layout.dim[0].size = 1
	msg_data.box_tag_position.layout.dim[0].stride = 3
	msg_data.box_tag_position.layout.dim[1].label = 'num_para'  # x, y, z
	msg_data.box_tag_position.layout.dim[1].size = 3
	msg_data.box_tag_position.layout.dim[1].stride = 3
	msg_data.box_tag_position.data = self.box_tag_position

        rospy.loginfo(msg_data)
        print('\n')
        self.tag_info_pub.publish(msg_data)
        self.rate.sleep()



if __name__ == '__main__':

    # set target areas by giving top left and bottom right coordinates of each area
    target_areas = {'left':[[0, -0.25], [0.25, 0]], 'right':[[0, 0.25], [0.25, 0.5]]}

    # set box tag ids
    box_ids = {'small':[0, 1, 2, 3, 4], 'big':[5, 6, 7, 8, 9]}

    # To Do: set receipt tag ids
    receipt_ids = []

    # publishing rate
    pub_rate =  50
    
    try:
	td = TagDetection(target_areas, box_ids, receipt_ids, pub_rate)
	rospy.spin()
    except KeyboardInterrupt:
        pass
