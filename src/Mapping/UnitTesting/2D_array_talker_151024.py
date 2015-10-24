# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 09:55:03 2015

@author: CKC
"""

#!/usr/bin/env python

import rospy
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

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(0,10):        
            hello_str = x[i]
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptionException:
        pass