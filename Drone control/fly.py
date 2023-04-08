#! /usr/bin/env python
import rospy
from drone_control.msg import MarkerPosition
from geometry_msgs.msg import Twist, PoseStamped

DRONE_HEIGHT = 0.3
isGoal = False
land = False
n = 0
x_pos = 4
y_pos = 4




def TakeoffLand():
    global landing, isGoal,twist_msg, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, twist_active

    rospy.init_node('fly') # Creates the node

    # Instance Variables
    landing = False # If quadcopter in landing mode or not- initially set to false unless otherwise
    isGoal = False # If quadcopter has a current goal to reach - initially set to false unless otherwise
    twist_active = True # If quadcopter is currenty being given a command velocity - initially set to true unless otherwise
    twist_msg = Twist() # Twist message to publish for hovering

    # Subcribers
    ardrone1_sub = rospy.Subscriber('/ardrone1/ground_truth_to_tf/pose', PoseStamped, takeoff_cb1) # Information about pose of the drone
    ardrone2_sub = rospy.Subscriber('/ardrone2/ground_truth_to_tf/pose', PoseStamped, takeoff_cb2) # Information about pose of the drone
    ardrone3_sub = rospy.Subscriber('/ardrone3/ground_truth_to_tf/pose', PoseStamped, takeoff_cb3) # Information about pose of the drone
    ardrone4_sub = rospy.Subscriber('/ardrone4/ground_truth_to_tf/pose', PoseStamped, takeoff_cb4) # Information about pose of the drone
    poses = rospy.Subscriber('/position', MarkerPosition, pose_cb)

    # Publishers
    ardrone1_cmd_vel_pub = rospy.Publisher('/ardrone1/cmd_vel', Twist, queue_size=1)
    ardrone2_cmd_vel_pub = rospy.Publisher('/ardrone2/cmd_vel', Twist, queue_size=1)
    ardrone3_cmd_vel_pub = rospy.Publisher('/ardrone3/cmd_vel', Twist, queue_size=1)
    ardrone4_cmd_vel_pub = rospy.Publisher('/ardrone4/cmd_vel', Twist, queue_size=1)
   
    rate = rospy.Rate(5) # rate at which to revisit callbacks
    rospy.spin() # keeps the node alive



def takeoff_cb1(msg):

    global landing, isGoal, twist_msg, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, land, x_pos, y_pos
    x_pos1 = msg.pose.position.x
    y_pos1 = msg.pose.position.y
    x_pose_taret = x_pos 
    y_pose_taret = y_pos
    #if x_pos >=3 and y_pos >=3:
    vel_x1 = x_pose_taret - x_pos1
    vel_y1 = y_pose_taret - y_pos1
    twist_msg.linear.x = vel_x1
    twist_msg.linear.y = vel_y1
    ardrone1_cmd_vel_pub.publish(twist_msg)

def takeoff_cb2(msg):

    global landing, isGoal, twist_msg, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, land, x_pos, y_pos
    x_pos2 = msg.pose.position.x
    y_pos2 = msg.pose.position.y
    x_pose_taret = x_pos -1
    y_pose_taret = y_pos
    #if x_pos >=3 and y_pos >=3:
    vel_x2 = x_pose_taret - x_pos2
    vel_y2 = y_pose_taret - y_pos2
    twist_msg.linear.x = vel_x2
    twist_msg.linear.y = vel_y2
    ardrone2_cmd_vel_pub.publish(twist_msg)

def takeoff_cb3(msg):

    global landing, isGoal, twist_msg, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, land, x_pos, y_pos
    x_pos3 = msg.pose.position.x
    y_pos3 = msg.pose.position.y
    x_pose_taret = x_pos 
    y_pose_taret = y_pos - 1
    #if x_pos >=3 and y_pos >=3:
    vel_x3 = x_pose_taret - x_pos3
    vel_y4 = y_pose_taret - y_pos3
    twist_msg.linear.x = vel_x3
    twist_msg.linear.y = vel_y4
    ardrone3_cmd_vel_pub.publish(twist_msg)

def takeoff_cb4(msg):

    global landing, isGoal, twist_msg, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, land, x_pos, y_pos
    x_pos4 = msg.pose.position.x
    y_pos4 = msg.pose.position.y
    x_pose_taret = x_pos 
    y_pose_taret = y_pos + 1
    #if x_pos >=3 and y_pos >=3:
    vel_x = x_pose_taret - x_pos4
    vel_y = y_pose_taret - y_pos4
    twist_msg.linear.x = vel_x
    twist_msg.linear.y = vel_y
    ardrone4_cmd_vel_pub.publish(twist_msg)

def pose_cb(msg):
  global x_pos, y_pos
  x_pos = msg.x
  y_pos = msg.y	                                                         

if __name__ == '__main__':
    TakeoffLand()
