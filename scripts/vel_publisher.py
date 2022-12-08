#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def publisher():

    
        
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    
    
    
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
        
        
