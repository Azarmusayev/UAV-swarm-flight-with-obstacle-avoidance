#!/usr/bin/env python
import numpy as np
import rospy, cv2, cv_bridge
import cv2.aruco as aruco
import sys, time, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from drone_control.msg import MarkerPosition
import numpy

#--- Define Tag
id_to_find  = 50
marker_size  = 10 #- [cm]

def main(msg):
    global pub, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub, x_pos, y_pos

    #--- Get the camera calibration path
    twist = Twist()
    poses = MarkerPosition()

    calib_path  = ""
    camera_matrix   = np.loadtxt("/home/hrithik/check_ws/src/drone/drone_control/src/cameraMatrix.txt", delimiter=',')
    camera_distortion   = np.loadtxt('/home/hrithik/check_ws/src/drone/drone_control/src/cameraDistortion.txt', delimiter=',')



    #--- 180 deg rotation matrix around the x axis
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters_create()


    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv_bridge.CvBridge()



    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    #while True:

    #-- Read the camera frame
    frame = cap.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    #cap = cv2.VideoCapture(frame)
    #ret, frame = cap.read(frame)

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    #corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    twist.linear.x = 1
    ardrone1_cmd_vel_pub.publish(twist)	
    ardrone2_cmd_vel_pub.publish(twist)
    ardrone3_cmd_vel_pub.publish(twist)
    ardrone4_cmd_vel_pub.publish(twist)	

    
    if ids is not None and ids[0] == id_to_find:
			print "Pad Found"
			while True:
				poses.x = x_pos
				poses.y = y_pos
				pub.publish(poses)

def land():
	global cmd_vel_pub, pub, ardrone1_cmd_vel_pub, ardrone2_cmd_vel_pub, ardrone3_cmd_vel_pub, ardrone4_cmd_vel_pub
	print "finding pad"
	pub = rospy.Publisher('position', MarkerPosition, queue_size=1)
	image_sub = rospy.Subscriber('/ardrone1/bottom/camera/image', Image, main)
	ardrone1_sub = rospy.Subscriber('/ardrone1/ground_truth_to_tf/pose', PoseStamped, pose_cb)
	ardrone1_cmd_vel_pub = rospy.Publisher('/ardrone1/cmd_vel', Twist, queue_size=1)
	ardrone2_cmd_vel_pub = rospy.Publisher('/ardrone2/cmd_vel', Twist, queue_size=1)
	ardrone3_cmd_vel_pub = rospy.Publisher('/ardrone3/cmd_vel', Twist, queue_size=1)
	ardrone4_cmd_vel_pub = rospy.Publisher('/ardrone4/cmd_vel', Twist, queue_size=1)

def pose_cb(msg):
	global x_pos, y_pos
	x_pos = msg.pose.position.x
	y_pos = msg.pose.position.y
	
if __name__ == '__main__':
		cv2.destroyAllWindows()
		rospy.init_node('landing')
		land()
		rospy.spin()