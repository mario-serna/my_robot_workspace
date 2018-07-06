#!/usr/bin/env python
import time
import  math
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import mover3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

MAX_FORWARD = 1.1
MAX_LEFT = 0.3
MIN_FORWARD = -1.1
MIN_LEFT = -0.3

forward = 0.0
left = 0.0
keyPress = 0

estado = [0,0,0]

mensaje = ""
flag = 1
def callback(data):
    global mensaje
    mensaje = data.pose.pose.position
    #rospy.loginfo("Nuevo")

def listener():
    #rospy.init_node('nodo_listener')
    rospy.Subscriber("odom", Odometry, callback)
       # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def odometria(ri,rd,estado,s,h=3):
    r = 0.4
    vl = (ri+rd)/2
    w = (1/r) * (rd-ri)/h
    #inx = vl * math.cos(math.radians(estado[2]))
    #iny = vl * math.sin(math.radians(estado[2]))

    inx = vl * math.cos(estado[2])
    iny = vl * math.sin(estado[2])
    estado[0] += inx * s
    estado[1] += iny * s
    estado[2] = (estado[2] + w * s) % (2 * math.pi)

    print estado

    return estado


"""while(keyPress != USER_QUIT):
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('p3dx_mover')

	twist = Twist()

	keyPress = getch.getArrow()

	if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
		forward += 0.1
	elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
		forward -= 0.1
	elif((keyPress == KEY_LEFT) and (left <= MAX_LEFT)):
		left += 0.1
	elif((keyPress == KEY_RIGHT) and (left >= MIN_LEFT)):
		left -= 0.1
"""
listener()

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('p3dx_mover')

twist = Twist()

print "Action 0"

time.sleep(1)

print "Action 1"

forward = 0.3
left = 0.0
twist.linear.x = forward
twist.angular.z = left
odometria(forward,forward,estado,4)
pub.publish(twist)
time.sleep(4)
rospy.loginfo("Odom 1 %s",mensaje)

print "Action 2"

forward = 0.0
left = 0.3
twist.linear.x = forward
twist.angular.z = left
odometria(-left,left,estado,3)
pub.publish(twist)
time.sleep(3)
rospy.loginfo("Odom 1 %s",mensaje)


print "Action 3"

forward = 0.3
left = 0.0
twist.linear.x = forward
twist.angular.z = left
odometria(forward,forward,estado,4)
pub.publish(twist)
time.sleep(4)
rospy.loginfo("Odom 1 %s",mensaje)
print "Action 4"

forward = 0.0
left = 0.3
twist.linear.x = forward
twist.angular.z = left
odometria(-left,left,estado,3)
pub.publish(twist)
time.sleep(3)
rospy.loginfo("Odom 1 %s",mensaje)


print "Terminado"

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('p3dx_mover')
twist = Twist()
pub.publish(twist)
exit()
