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

target=180
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

goal.x=[-18.37]
goal.y=[19.24]
for i in range(1):
        arrived = False
        while not arrived:
                inc_x = goal.x[i] -x
                inc_y = goal.y[i] -y
                angle_to_goal = arctan2(inc_y, inc_x)
                angle_deg=arctan2(inc_y,inc_x) * 180/np.pi

                # print(angle_deg)

                if angle_deg > 90:
                    val = 0.45
                else:
                    val = -0.45

                if abs(angle_to_goal - theta) > 0.4:
                    speed.angular.z = val
                    speed.linear.x = 0.0
                else:
                    speed.linear.x = 1.0
                    speed.angular.z = 0.0

                if abs(inc_x) <1 and abs(inc_y)< 1:
                    arrived=True
                    print("reached")
                    speed.linear.x=0.0
                    speed.angular.z=0.0
                pub.publish(speed)
       


goTo(goal)


