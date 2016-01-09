#!/usr/bin/env python


import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *

def callback(data):
    newarray = []
    array = data.data
    for x in range(0, len(array)):
        inVal = array[x]
        newarray.append(inVal+1)
    
    print(newarray)

def subscriber():
   
    rospy.init_node('testy', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback)
    

    rospy.spin()

if __name__ == '__main__':
    subscriber()
