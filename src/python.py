#!/usr/bin/env python
import rospy
from std_msgs.msg import String

#SOLUTION - DO NOT COPY
primes = []
def is_prime(n):
	result = reduce(lambda prime_so_far, num: prime_so_far and (n % num != 0),
		primes,
		True)
	if result:
		primes.append(n)
	return result

pub = rospy.Publisher('primes', String)
rospy.init_node('py', anonymous=True)

for n in range(1,101):
	fizzbuzz = False
	is_n_prime = is_prime(n)
	message = ''
	if is_n_prime:
		message += 'prime'
	
	if n % 3 == 0 and not is_n_prime:
		fizzbuzz = True
		message += 'fizz'
	if n % 5 == 0 and not (is_n_prime and fizzbuzz):
		fizzbuzz = True
		message += 'buzz'
	if not fizzbuzz and not is_n_prime:
		message += str(n)
	rospy.loginfo(message)
	pub.publish(message)

