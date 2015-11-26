#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import String

x = np.zeros((10,10), dtype='int')

for i in range(0,10):
    x[i, 1] = 1

for i in range(0,10):
    x[i, 8] = 1

for i in range(2, 4):
    for j in range(3, 5):
        x[i, j] = 1
        
for i in range(6, 8):
    for j in range(4,6):
        x[i, j] = 1

def unit_test_node():
	test_map = OccupancyGrid()
	test_map.data = []
	for i in range(0, 10):
		test_map.data.append(x[i])	
	pub = rospy.Publisher('unit_tests', OccupancyGrid, queue_size=10)
	rospy.init_node('unit_test_node', anonymous = True)
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		rospy.loginfo(test_map)
		pub.publish(test_map)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        unit_test_node()
    except rospy.ROSInterruptException:
        pass
