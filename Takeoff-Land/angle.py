#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from numpy import arctan2
import numpy as np

x = 0
y = 0
theta = 0

target=90
kp=0.5

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller2")
sub = rospy.Subscriber("/uav2/ground_truth/state", Odometry, newOdom)
pub = rospy.Publisher("/uav2/cmd_vel", Twist, queue_size = 1)
r=rospy.Rate(10)

speed = Twist()

goal = Point()
# goal.x = [6.466,6.466,5.5,5.5,7.73,6,3,-1,-8.0]
# goal.y = [-8.743,-6.525,-4.617,-2.681,0.948,3.00,4.00,5.629,5.629]

goal.x = [-8.0]
goal.y = [5.629]

def goTo(goal):
    global x
    global y
    global theta
    global sub
    global pub
    global speed
    global target
    global kp

    # target_rad=target * np.pi/180

    while True:
        target_rad=target * np.pi/180

        if((target_rad-theta)<0.01):
            speed.angular.z=0
            pub.publish(speed)
            print("yes")
            break
       
        speed.angular.z=kp*(target_rad-theta)
        print(target_rad," ",theta)
        pub.publish(speed)
        r.sleep()


goTo(goal)


