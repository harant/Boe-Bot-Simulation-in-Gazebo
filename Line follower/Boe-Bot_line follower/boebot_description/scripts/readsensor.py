#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def go():
	msg=True
	pub.publish(msg)

def stop():
	msg=False
	pub.publish(msg)


def clbk_laser(msg):

	distance = min( min(msg.ranges[0:10]), 0.10)
	#rospy.loginfo(distance)

	if (distance == 0.1):
		go()

	else:
		stop()



if __name__ == "__main__":
	rospy.init_node('reading_sensors')

	sub = rospy.Subscriber('autobot/obstacle_sensor', LaserScan, clbk_laser)
	pub = rospy.Publisher('/can_robot_go',Bool,queue_size=1)

	rospy.spin()