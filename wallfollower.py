#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np

import math

active_ = False
pub_ = None

sensor_right = 0.0
sensor_front = 0.0 
sensor_left = 0.0

state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

close_distance = 0.075

def clbk_laser_right(msg):

    global sensor_right

    sensor_right = min(min(msg.ranges[0:10]),10)


def clbk_laser_front(msg):

    global sensor_front

    sensor_front = min(min(msg.ranges[0:10]),10)


def clbk_laser_left(msg):

    global sensor_left

    sensor_left = min(min(msg.ranges[0:10]),10)

    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    global sensor_left
    global sensor_front
    global sensor_right

    msg = Twist()

    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.25
    
    if sensor_front > d and sensor_left > d and sensor_right > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif sensor_front < d and sensor_left > d and sensor_right > d:
        state_description = 'case 3 - front'
        change_state(1)
    elif sensor_front > d and sensor_left > d and sensor_right < d:
        state_description = 'case 4 - fright'
        change_state(2)
    elif sensor_front > d and sensor_left < d and sensor_right > d:
        state_description = 'case 5 - fleft'
        change_state(0)
    elif sensor_front < d and sensor_left > d and sensor_right < d:
        state_description = 'case 6 - front and fright'
        change_state(1)
    elif sensor_front < d and sensor_left < d and sensor_right > d:
        state_description = 'case 7 - front and fleft'
        change_state(1)
    elif sensor_front < d and sensor_left < d and sensor_right < d:
        state_description = 'case 8 - front and fleft and fright'
        change_state(1)
    elif sensor_front > d and sensor_left < d and sensor_right < d:
        state_description = 'case 9 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.5
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():

    base_vel = 0.3
    
    global sensor_right
    global close_distance
    msg = Twist()
    
    
    if sensor_right < close_distance:
        msg.angular.z = base_vel
    else:
        msg.linear.x = base_vel
        
    return msg

def main():
    global pub_
    
    rospy.init_node('wallfollower')
    
    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #Subscribing to all the sensor topics
    sub1 = rospy.Subscriber('autobot/obstacle_sensor_right', LaserScan, clbk_laser_right)
    sub2 = rospy.Subscriber('autobot/obstacle_sensor_front', LaserScan, clbk_laser_front)
    sub3 = rospy.Subscriber('autobot/obstacle_sensor_left', LaserScan, clbk_laser_left)
        
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()