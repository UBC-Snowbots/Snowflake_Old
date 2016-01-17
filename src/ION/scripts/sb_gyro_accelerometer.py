#!/usr/bin/python

#Author: Vincent Yuan, modified by Gareth Ellis 
#Purpose: serial adapter, because c++ is too hard to serial communication 
#Date: May 17, 2015 / Modified Jan 16, 2015
#Reminder: chmod +x sb_gyro_accelerometer.py 
#Run: rosrun ION sb_gyro_accelerometer.py 
import serial
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion Vector3
from std_msgs.ms import string
import math
	
def main ():
        pub = rospy.Publisher ('imu/data', string, queue_size=10)
	rospy.init_node('sb_gyro_accelerometer')
	rate = rospy.Rate(10) # 10HZ
	link = serial.Serial(port="/dev/ttyACM0",baudrate=1000000)
	while not rospy.is_shutdown():
		rospy.loginfo(link.readline())
		pub.publish(link.readline())
		rate.sleep()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass

