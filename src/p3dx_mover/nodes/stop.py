#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('p3dx_mover')
import rospy

from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('p3dx_mover')
twist = Twist()
twist.linear.x = 0.0
twist.angular.z = 0.0
pub.publish(twist)
print "Stop"
exit()
