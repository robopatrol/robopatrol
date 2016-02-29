#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist




def main():
    print "Run robo patrol"

    rospy.init_node('robopatrol')

    while True:
        speed = input("Speed: ")
        distance = input("Distance: ")
        move(speed, distance)
    

def move(speed, distance):
    p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    t0 = rospy.rostime.get_time()
    current_distance = 0

    while current_distance < distance:
        p.publish(twist)
        t1 = rospy.rostime.get_time()
        current_distance = abs(speed * (t1 - t0))

    twist.linear.x = 0
    p.publish(twist)



if __name__ == "__main__":
    main()
