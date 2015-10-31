#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import String
fizzbuzz = False

def isprime(n):
    if n == 2: return True
    if n % 2 == 0: return False
    else:
        for i in range(3, int(n**0.5)+1, 2):
            if n % i == 0:
                return False
        return True

def talker(): 
	pub = rospy.Publisher('primes', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(50)
	for i in range(100):
		fizzbuzz = False
		fizzstring = str(i)+"->"
		if isprime(i):
			fizzstring += "Prime"
			fizzbuzz = True
		if i % 3 == 0:
			fizzstring += "Fizz"
			fizzbuzz = True
		if i % 5 == 0:
			fizzstring += "Buzz"
			fizzbuzz = True
		if not fizzbuzz:
			fizzstring += str(i)
		rospy.loginfo(fizzstring)
		pub.publish(fizzstring)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

