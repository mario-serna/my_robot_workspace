import roslib; roslib.load_manifest('p3dx_mover')
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo("I heard %s",data.child_frame_id)

def listener():
    rospy.init_node('nodo_listener')
    rospy.Subscriber("odom", Odometry, callback)
       # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()