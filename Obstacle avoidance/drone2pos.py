#!/usr/bin/env python

# for degrees()
import math

# import ros related libraries
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#global variables
global pub_vel
global odom_sub
global rob_goal_pos
global prev_time
global stop


# robot goal - x pos, y pos , z pos
rob_goal_pos = [-1.0, -1.0, -1.0]

#previous time
prev_time = 0.0

#robot stop
stop = False


def odom_callback(msg):
    """ ros subscriber callback"""

    global stop

    if not stop:
        #store robot x,y,z linear position
        x_pos = round(msg.pose.pose.position.x, 3)
        y_pos = round(msg.pose.pose.position.y, 3)
        z_pos = round(msg.pose.pose.position.z, 3)
        
        #robot-                    x position, y postion,z position
        print('robot current position =', x_pos, y_pos, z_pos)

        #robot controller trigger every time ros subscriber callback is called
        move2goal(x_pos, y_pos, z_pos)

def publish_on_vel(x_vel, y_vel, z_vel):
    """ function to publish on a ros topic"""

    global pub_vel

    #publish velocity
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = x_vel
    cmd_vel_msg.linear.y = y_vel
    cmd_vel_msg.linear.z = z_vel
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    pub_vel.publish(cmd_vel_msg)


def move2goal(curr_x, curr_y, curr_z):
    """ function to make a PD controller 
    & calculate the x,y linear velocity & roation about z angular velocity"""

    global rob_goal_pos
    global prev_time
    global stop
    

    if not stop:

        #error difference in robot state
        e_x = rob_goal_pos[0] - curr_x
        e_y = rob_goal_pos[1] - curr_y
        e_z = rob_goal_pos[2] - curr_z

        #time
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - prev_time
        print(dt)

        #change in difference taking previous state to 0
        d_x = (e_x) / dt
        d_y = (e_y) / dt
        d_z = (e_z) / dt


        #PD coefficient
        Kp = 3.0   #adjust value
        Kd = 2.0   #adjust value
        
        #PD controller 
        #final velocity = kp x error + kd x rate of change of error
        f_x = Kp * e_x + Kd * d_x  
        f_y = Kp * e_y + Kd * d_y 
        f_z = Kp * e_z + Kd * d_z 

        #factor to make drone motion slower
        rate = 0.3

        #publish on ros topic 
        publish_on_vel(rate*f_x, rate*f_y, rate*f_z)

        #store all previous values
        prev_time = curr_time
        
        #goal tolerance
        tol_x = 0.05
        tol_y = 0.05
        tol_z = 0.05
        
        #if goal reached than stop
        if(abs(rob_goal_pos[0] - curr_x) < tol_x):
            if(abs(rob_goal_pos[1] - curr_y) < tol_y):
                if(abs(rob_goal_pos[2] - curr_z) < tol_z):
                    print("Goal Reached!")
                    publish_on_vel(0,0,0)
                    stop = True
        
        

def ros_init():
    """ function to initize ros node & ros publisher subscriber"""

    global pub_vel, rob_goal_pos, odom_sub

    rospy.init_node('drone_move2pos', anonymous=True)

    rob_goal_pos[0] = input("Enter Goal X pos: ")
    rob_goal_pos[1] = input("Enter Goal Y pos: ")
    rob_goal_pos[2] = input("Enter Goal Z pos: ")
    
    #ros publisher
    pub_vel = rospy.Publisher('drone/cmd_vel', Twist, queue_size=10)

    #ros subscriber
    odom_sub = rospy.Subscriber('drone/odom', Odometry, odom_callback)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    rospy.spin()

if __name__ == '__main__':

    try:
        ros_init()
    except rospy.ROSInterruptException:
        pass