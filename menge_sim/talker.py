#!/usr/bin/env python

import rospy 
from std_msgs.msg import String 

from geometry_msgs.msg import Twist 


def talker():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)   ## I am not sure about queue size
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)  

	msg = Twist()
	count = 0
	while not rospy.is_shutdown():
		
		# if (count % 2 == 0):
		# 	msg.linear.x = 0.5   ## prefVelMsg.setSpeed(msg.linear.x)
		# 	msg.angular.z = 0    ## prefVelMsg.turn(msg.angular.z)
		# else:
		# 	msg.linear.x = 0.0
		# 	msg.angular.z = 0.5
		msg.linear.x = 0.0
		rospy.loginfo("checking for cmd \n" + str(msg.linear.x) + " " + str(msg.angular.z))
		pub.publish(msg)

		count += 1
		rate.sleep()

if __name__ == '__main__':

	try:
		talker()
	except rospy.ROSInterruptException:
		pass

## change talker.py to publish /cmd_vel 
## add a subscriber in talkerpy that reads the laser end data  => right now, separte file listener.py  