#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *


#def callback(data):
    
    #newArray = data.data

def mapMethod():
    pub = rospy.Publisher('mymap', OccupancyGrid, queue_size=10)
    rospy.init_node('saved_maps_node', anonymous=True)
    #rospy.Subscriber("global_map", OccupancyGrid, callback)
    
    
    X_VALUE = 2
    Y_VALUE = 2
    rate = rospy.Rate(0.5) #delay time (times per second)

    while not rospy.is_shutdown():
        
        
        test_map = OccupancyGrid()
        
        test_map.info.width = X_VALUE
        test_map.info.height = Y_VALUE
        test_map.info.origin.position.x = 0
        test_map.info.origin.position.x = 0
        array = []
        for i in range(0, X_VALUE * Y_VALUE):
            array.append(1) 

        test_map.data = array

        pub.publish(test_map)

        print(test_map.data)
        rate.sleep()
        #rospy.spin()




if __name__ == '__main__':
    try: 
        mapMethod()
    except rospy.ROSInterruptException:
        pass
