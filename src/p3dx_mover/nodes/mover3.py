#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('p3dx_mover')
import rospy

from nav_msgs.msg import Odometry

estado = []
flag = 1
def callback(data):
    global estado
    estado = data.pose.pose.position
    #rospy.loginfo("Nuevo")

def listener():
    rospy.init_node('nodo_listener')
    rospy.Subscriber("odom", Odometry, callback)
       # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

"""listener()
while (flag==1):
    rospy.loginfo("I heard %s",mensaje)
    time.sleep(1);"""