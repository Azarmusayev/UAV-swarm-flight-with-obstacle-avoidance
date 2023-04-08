#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

x_2=0

def state_callback(pose):

    # rospy.loginfo(pose)
    global x_2
    x_2=pose.pose.pose.position.z
    # print(x)

def ini():
    global x_2
    twist_msg=Twist()
    twist_msg.linear.z=0.4
    print(x_2)

    while True:

        print("loop")
        if x_2<3.54:
            cmd_vel_pub.publish(twist_msg)
            print(x_2)
        else:
            print("end")
            twist_msg.linear.z=0
            cmd_vel_pub.publish(twist_msg)
            break


    rate = rospy.Rate(10)
    rospy.spin()
    

if __name__=='__main__':

    rospy.init_node('takoff3')

    state=rospy.Subscriber('/uav2/ground_truth/state',Odometry,state_callback)
    cmd_vel_pub=rospy.Publisher('/uav2/cmd_vel',Twist,queue_size=5)
    ini()

    