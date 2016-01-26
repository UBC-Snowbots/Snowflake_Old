#!/usr/bin/python

#Author: Vincent Yuan, modified by Gareth Ellis 
#Purpose: serial adapter, because c++ is too hard to serial communication 
#Date: May 17, 2015 / Modified Jan 16, 2015
#Reminder: chmod +x sb_gyro_accelerometer.py 
#Run: rosrun ION sb_gyro_accelerometer.py 
import serial
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import math
	
def main ():
    pub = rospy.Publisher ('imu/data', Imu, queue_size=10)
    rospy.init_node('sb_gyro_accelerometer')
    rate = rospy.Rate(10) # 10HZ
    link = serial.Serial(port="/dev/ttyACM1",baudrate=9600)
    seqID = 0 #Sequently increasing ID for header in IMU
    h = Header()
	
	# For each section of the IMU (o, a_v, and l_a) if there is no 
	# data produced, set the first element of the associated covariance
	# to -1
    o = Quaternion() # Orientation
    o_c = [-1,0,0,0,0,0,0,0,0]         # Orientation Covariance
    a_v = Vector3()                    # Angular Velocity
    a_v_c = [-1,0,0,0,0,0,0,0,0]	   # Angular Velocity Covariance
    l_a = Vector3()                    # Linear Acceleration
    l_a_c = [-1,0,0,0,0,0,0,0,0]        # Linear Acceleration Covariance

    while not rospy.is_shutdown():
        try:
            #rospy.loginfo(link.readline())
            raw_data = link.readline()
            data = []
            for value in raw_data.split(',')[0:5]:
                if value[0] == '-':
                    data.append(-1 * float(value[1:]))
                else:
                    data.append(float(value))
			# Create Header
            h.seq = seqID
            seqID += 1
            h.stamp = rospy.Time.now()
            h.frame_id = str(0)
			
			# Add values to orientation (Quaternion)
            # Sum of squares of all 4 values must add up to 1
            #o.x = 1 / math.sqrt(1 + math.pow(data[4], 2))
            o.x = math.cos(data[4]) / math.sqrt(math.pow(math.cos(data[4]), 2) + math.pow(math.sin(data[4]), 2))
            o.y = math.sin(data[4]) / math.sqrt(math.pow(math.cos(data[4]), 2) + math.pow(math.sin(data[4]), 2))
            o.z = 0
            o.w = 0

			# Add values to angular velocity (Vector3)
            a_v.x = 0
            a_v.y = 0
			#a_v.z = data[3] * math.pi/180  # Make sure to convert to rad
            a_v.z = 0;

			# Add values to linear acceleration,
            # Make sure to convert all to m/s^2
            l_a.x = data[0] * (9.8 * 10**-6)
            l_a.y = data[1] * (9.8 * 10**-6)
            l_a.z = data[2]	* (9.8 * 10**-6)

            pub.publish(h, o, o_c, a_v, a_v_c, l_a, l_a_c)
            rate.sleep()
        except:
            print("ERROR")
            pass
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass

