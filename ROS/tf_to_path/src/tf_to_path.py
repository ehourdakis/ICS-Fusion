#!/usr/bin/env python

import rospy
import tf
import rotations 

import std_msgs
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose

path_pub = None
path = Path()


def getPose(trans,rot):
    pose = Pose()
    pose.position.x=trans[0]
    pose.position.y=trans[1]
    pose.position.z=trans[2]
    
    pose.orientation.x=rot[0]
    pose.orientation.y=rot[1]
    pose.orientation.z=rot[2]
    pose.orientation.w=rot[3]
    return pose
    
    
rospy.init_node('tf_to_path', anonymous=True)
src_frame = None
dst_frame = None
try:
    src_frame = rospy.get_param("~src_frame")
    dst_frame = rospy.get_param("~dst_frame")  
except Exception as  e:
    print("could not get parameters")
    exit(1)    

path_pub = rospy.Publisher('/gtpath', Path, queue_size=1000)
rate = rospy.Rate(200.0)
listener = tf.TransformListener()

initial_trans = None
initial_rot = None 
initial_pose = None
while not rospy.is_shutdown():
    
    try:
        (initial_trans,initial_rot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    initial_pose = getPose(initial_trans,initial_rot)
    print(initial_trans)
    break

while not rospy.is_shutdown():
    
    try:
        (trans,rot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    time =  rospy.Time.now()
    #print(trans)
    
    
    path.header=std_msgs.msg.Header()
    path.header.frame_id='odom'
    path.header.stamp= time
    
    pose = PoseStamped()
    pose.header=std_msgs.msg.Header()
    pose.header.frame_id='odom'
    pose.header.stamp= time
    
    
    #pose.pose.position.x=trans[0] - initial_trans[0]
    #pose.pose.position.y=trans[1] - initial_trans[0]
    #pose.pose.position.z=trans[2] - initial_trans[0]
    
    #pose.pose.orientation.x=rot[0]
    #pose.pose.orientation.y=rot[1]
    #pose.pose.orientation.z=rot[2]
    #pose.pose.orientation.w=rot[3]
    pose_tmp=getPose(trans,rot)
    pose_tmp.position.x=pose_tmp.position.x-initial_trans[0]
    pose_tmp.position.y=pose_tmp.position.y-initial_trans[1]
    pose_tmp.position.z=pose_tmp.position.z-initial_trans[2]
    #initial_pose
    #pose.pose=rotations.transform(initial_pose,pose_tmp)
    pose.pose=pose_tmp
    
    path.poses.append(pose)    
    path_pub.publish(path)
#try:
    #fileName = rospy.get_param("~file_name")    
    #rbgStr =rospy.get_param("~rgb")
    #originStr = rospy.get_param("~origin")
    
    #rgbList = rbgStr.split(',')    
    #rgb[0] = int(rgbList[0])
    #rgb[1] = int(rgbList[1])
    #rgb[2] = int(rgbList[2])
    
    #originList = originStr.split(',')
    #origin[0] = int(originList[0])
    #origin[1] = int(originList[1])
    #origin[2] = int(originList[2])
#except Exception as  e:
    #print("could not get parameters")
    #exit(1)    
    
#try:
    #pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
    #rate = rospy.Rate(10) # 10hz
    
    #readFile(fileName)
    #while not rospy.is_shutdown():
        #publishMarkers()
        #rate.sleep()
#except rospy.ROSInterruptException:
    #pass
