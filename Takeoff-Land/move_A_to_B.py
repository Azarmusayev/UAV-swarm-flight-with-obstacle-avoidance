#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from numpy import arctan2
import numpy as np
import time

x = 0
y = 0
z=0
theta = 0

target=180
target1=90
kp=0.5

def newOdom(msg):
    global x
    global y
    global z
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z=msg.pose.pose.position.z

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller2")
sub = rospy.Subscriber("/uav2/ground_truth/state", Odometry, newOdom)
pub = rospy.Publisher("/uav2/cmd_vel", Twist, queue_size = 1)

speed = Twist()

goal = Point()
goal.x = [6.466,6.466,5.5,5.5,7.73,6,3,1]
goal.y = [-8.743,-6.525,-4.617,-2.681,0.948,3.00,4.00,5.629]

# goal.x = [-17.0]
# goal.y = [5.629]

def goTo(goal):
    global x
    global y
    global z
    global theta
    global sub
    global pub
    global speed
    global target
    global target1
    global kp

  

    for i in range(8):

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

    goal.x=-8

    while True:
        if x>-8:
            speed.linear.x=0.9
            speed.angular.z=0.0
            pub.publish(speed)
        else:
            speed.linear.x=0
            pub.publish(speed)
            break
    while True:
        if z>1.96:
            speed.linear.z=-0.4
            speed.linear.x=0.7
            pub.publish(speed)
        else:
            speed.linear.z=0
            speed.linear.x=0
            pub.publish(speed)
            break
    while True:
        if x>-15.5:
            print("going")
            speed.linear.x=1.2
            speed.angular.z=0.0
            pub.publish(speed)
        else:
            speed.linear.x=0
            pub.publish(speed)
            print("rech")
            break
    # time.sleep(1.5)
    
    goal.x=[-20.1,-18.37]
    goal.y=[9.45,13.73]
    for i in range(2):
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
        # target1=90
    while True:
        target_rad=target1 * np.pi/180
        if((target_rad-theta)<0.01):
            speed.angular.z=0
            pub.publish(speed)
            print("yes")
            break
       
        speed.angular.z=kp*(target_rad-theta)
        print(target_rad," ",theta)
        pub.publish(speed)
    while True:
        if y<20:
            speed.linear.x=1.3
            speed.angular.z=0.0
            pub.publish(speed)
        else:
            speed.linear.x=0
            pub.publish(speed)
            break
    while True:
        if x>-22:
            speed.linear.y=0.5
            speed.angular.z=0.0
            pub.publish(speed)
        else:
            speed.linear.y=0
            pub.publish(speed)
            break
    while True:
        if z>0.3:
            speed.linear.z=-0.4
            speed.linear.x=0
            pub.publish(speed)
        else:
            speed.linear.z=0
            speed.linear.x=0
            pub.publish(speed)
            break

goTo(goal)


