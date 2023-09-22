#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

previous_x=0

def call_set_pen_service(r,g,b,width,off):
    try:
        set_pen=rospy.ServiceProxy("/turtle1/set_pen",SetPen)
        rosponse=set_pen(r,g,b,width,off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def call(pose:Pose):
    cmd=Twist()
    if pose.x<2.0 or pose.x>9.0 or pose.y<2.0 or pose.y>9.0:
        cmd.linear.x=1.0
        cmd.angular.z=1.4
    #lif pose.x>5.0:
       # call_set_pen_service(0,255,0,3,0)
    else:
        cmd.linear.x=4.0
        cmd.angular.z=0.0

    global previous_x

    if pose.x>=5.5 and previous_x<5.5:
        previous_x=pose.x
        rospy.loginfo("set color to red!")
        call_set_pen_service(255,0,0,3,0)
    elif pose.x<5.5 and previous_x>=5.5:
        previous_x=pose.x
        rospy.loginfo("set color to green!")
        call_set_pen_service(0,255,0,3,0)


    pub.publish(cmd)

if __name__=='__main__':
    rospy.init_node("controller")
    rospy.wait_for_service("/turtle1/set_pen")

    call_set_pen_service(255,0,0,3,0)
    rospy.loginfo("node is starting")

    pub=rospy.Publisher("turtle1/cmd_vel",Twist,queue_size=10)
    sub=rospy.Subscriber("turtle1/pose",Pose,callback=call)
    rospy.spin()