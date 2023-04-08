#!/usr/bin/env python
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist
from tf.transformations import quaternion_from_euler
import numpy as np
from tf.transformations import euler_from_quaternion
from numpy import arctan2

theta = 0
def state_callback(pose):

    # rospy.loginfo(pose)
    global x
    global theta
    x=pose.pose.pose.position.z
    rot_q = pose.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # print(x)

#this method will make the robot move to the goal location
def move_to_goal(a,b):
   rospy.init_node('map_navigation', anonymous=False)
   #define a client for to send goal requests to the move_base server through a SimpleActionClient
   ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)


   state=rospy.Subscriber('/ground_truth/state',Odometry,state_callback)
   md_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)

   twist_msg=Twist()
   twist_msg.linear.z=-0.4
   #wait for the action server to come up
   while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the move_base action server to come up")

   goal = MoveBaseGoal()
   
   
   #set up the frame parameters
   goal.target_pose.header.frame_id = "map"
   goal.target_pose.header.stamp = rospy.Time.now()

   # moving towards the goal

   
   

   goal.target_pose.pose.position =  Point(a,b,0)

   quaternion = quaternion_from_euler(0,0, 120)
   
   goal.target_pose.pose.orientation.x = quaternion[0]
   goal.target_pose.pose.orientation.y = quaternion[1]
   goal.target_pose.pose.orientation.z = quaternion[2]
   goal.target_pose.pose.orientation.w = 1

   rospy.loginfo("Sending goal location ...")
   
   ac.send_goal(goal)

   ac.wait_for_result()

   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("You have reached the destination")
        #    return True

   else:
           rospy.loginfo("The robot failed to reach the destination")
        #    return False

   

#    goal.target_pose.pose.position =  Point(-27.5,18,0)
#    ac.send_goal(goal)


if __name__ == '__main__':
        state=rospy.Subscriber('/ground_truth/state',Odometry,state_callback)
        md_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)

        twist_msg=Twist()
        # twist_msg.linear.z=-0.4

        move_to_goal(-7.5,5.5)
        target=180
        kp=0.5
        while True:
                target_rad=target * np.pi/180
                if((target_rad-theta)<0.01):
                        twist_msg.angular.z=0
                        md_vel_pub.publish(twist_msg)
                        print("yes")
                        break
        
                twist_msg.angular.z=kp*(target_rad-theta)
                print(target_rad," ",theta)
                md_vel_pub.publish(twist_msg)

        while True:

                if x>1.96:
                        twist_msg.angular.z=0
                        twist_msg.linear.z=-0.4
                        md_vel_pub.publish(twist_msg)
                        print(x)
                else:
                        twist_msg.linear.z=0
                        md_vel_pub.publish(twist_msg)
                        break
        move_to_goal(-19.48,18)

        while True:
                if x>0.3:
                        twist_msg.linear.z=-0.4
                        twist_msg.linear.x=0
                        md_vel_pub.publish(twist_msg)
                else:
                        twist_msg.linear.z=0
                        twist_msg.linear.x=0
                        md_vel_pub.publish(twist_msg)
                        break
#    rospy.init_node('map_navigation', anonymous=False)
#    print('start go to goal')
# #    goal_x=[-9.1,10.7,12.6,12.6,12.6,18.2,-2]
# #    goal_y=[-1.2,10.5,4.2,-1.80,4.2,-1.4,4]
# #    goal_th=[180,270,-180,92,0,90,0]
# #    goal_x=[3.083,-1.033,3.088,-1.105,3.088,-1.0578,3.088]
# #    goal_y=[0.018,1.293,2.236,3.295,4.1686,4.971,5.081]
# #    goal_th=[180,270,-180,92,0,90,0]
#    goal_x=[0.510289,-1.4068]
#    goal_y=[-1.0928,-0.61154]
#    goal_th=[180,270]
#    for i in range(2):
#     move_to_goal(goal_x[i],goal_y[i],goal_th[i])
    
#    #move_to_goal(-9.1,-1.2, math.radians(180))
#    #move_to_goal(10.7,10.5, math.radians(270))
#    #move_to_goal(12.6,4.2, math.radians(-180))
#   # move_to_goal(12.6,-1.85,math.radians(92))
#    #move_to_goal(12.6,4.2, math.radians(-180))

   
#    #move_to_goal(18.2,-1.4, 90)
#    #move_to_goal(-2,4, 0)

#    #rospy.spin()
