#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

x=0
x_3=0
z=0

def state_callback(pose):

    # rospy.loginfo(pose)
    global x
    x=pose.pose.pose.position.z
    # print(x)

def state_callback1(msg):
    global x_3
    x_3=msg.pose.pose.position.z

def state_callback2(msg):
    global z
    z=msg.pose.pose.position.z

if __name__=='__main__':

    rospy.init_node('takoff')

    state=rospy.Subscriber('/uav2/ground_truth/state',Odometry,state_callback)
    state2=rospy.Subscriber('/uav3/ground_truth/state',Odometry,state_callback1)
    state3=rospy.Subscriber('/ground_truth/state',Odometry,state_callback2)
    cmd_vel_pub1=rospy.Publisher('/uav1/cmd_vel',Twist,queue_size=5)
    cmd_vel_pub2=rospy.Publisher('/uav2/cmd_vel',Twist,queue_size=5)
    cmd_vel_pub3=rospy.Publisher('/uav3/cmd_vel',Twist,queue_size=5)
    cmd_vel_pub4=rospy.Publisher('/cmd_vel',Twist,queue_size=5)

    twist_msg=Twist()
    twist_msg1=Twist()
    twist_msg2=Twist()
    twist_msg.linear.z=0.4
    twist_msg1.linear.z=0.4
    twist_msg2.linear.z=0.4
    while True:

        if x_3<1.80:
            cmd_vel_pub3.publish(twist_msg1)
        
        else:
            twist_msg1.linear.z=0
            cmd_vel_pub3.publish(twist_msg1)

        if z<3.1:
            cmd_vel_pub4.publish(twist_msg2)
        
        else:
            twist_msg2.linear.z=0
            cmd_vel_pub4.publish(twist_msg2)

        if x<3.47:
            cmd_vel_pub1.publish(twist_msg)
            cmd_vel_pub2.publish(twist_msg)
            # cmd_vel_pub4.publish(twist_msg)
            print(x)
        else:
            twist_msg.linear.z=0
            cmd_vel_pub1.publish(twist_msg)
            cmd_vel_pub2.publish(twist_msg)
            # cmd_vel_pub3.publish(twist_msg)
            # cmd_vel_pub4.publish(twist_msg)
            break


    rate = rospy.Rate(10)
    rospy.spin()
