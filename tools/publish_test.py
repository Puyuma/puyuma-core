#!/usr/bin/env python

import rospy

from xenobot.msg import segment
from xenobot.msg import segmentArray
from numpy import random
import math

segment_data_array = None
data_size = 50
hz = 5

def data_gen():
	global segment_data_array
	segment_data_array = segmentArray()
	for i in range(0, data_size, 1):
		segment_data = segment(d=-25 + 50*random.rand(),phi=180*random.rand(),color=math.floor(random.rand()*2))
		#segment_data.data = (d=2.0,phi=1.0,color=0)
		segment_data_array.segments.append(segment_data)

def main():
	rospy.init_node('publish_test',anonymous=True)

	topic_name = "/xenobot/segment_data"

	pub = rospy.Publisher(topic_name,segmentArray,queue_size=5)
	rate = rospy.Rate(hz)
	while not rospy.is_shutdown():
		data_gen()
		pub.publish(segment_data_array)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
