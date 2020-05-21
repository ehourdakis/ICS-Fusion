#!/usr/bin/env python

import rospy
import tf
import rotations 

import std_msgs
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
path_pub = None
path = Path()
initialPoseMat = None
baseToCamMat = None

def publishPath(pose, stamp):
    global path, path_pub
    path.header=std_msgs.msg.Header()
    path.header.frame_id = 'odom'
    path.header.stamp = stamp
    
    poseStamped = PoseStamped()
    poseStamped.header=std_msgs.msg.Header()
    poseStamped.header.frame_id='odom'
    poseStamped.header.stamp= stamp

    poseStamped.pose = pose
    path.poses.append(poseStamped)
    path_pub.publish(path)


def gtCallback(gtOdom):
    global initialPoseMat
    pose = gtOdom.pose.pose
    print(pose)
    
    if initialPoseMat is None:
        initialPoseMat = rotations.homogeneous(pose)
        M = np.dot(initialPoseMat,baseToCamMat)
        #M = np.dot(baseToCamMat,initialPoseMat)
        
        initialPoseMat = np.linalg.inv(M) 
        
    print(initialPoseMat)
    
    poseMat = rotations.homogeneous(pose)
    M = np.dot(initialPoseMat,poseMat)
    #M = np.dot(poseMat,initialPoseMat)
    
    
    pose = rotations.poseFromHomo(M)
    publishPath(pose, gtOdom.header.stamp)

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
    gt_topic = rospy.get_param("~gt_topic") 
except Exception as  e:
    print("could not get parameters")
    exit(1)    

path_pub = rospy.Publisher('/gtpath', Path, queue_size=1000)
rate = rospy.Rate(200.0)
listener = tf.TransformListener()


while not rospy.is_shutdown():

    try:
        (baseToCamTrans,baseToCamRot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    baseToCamPose = getPose(baseToCamTrans,baseToCamRot)  
    baseToCamMat = rotations.homogeneous(baseToCamPose)
    baseToCamMat =  np.linalg.inv(baseToCamMat) 
    break

rospy.Subscriber(gt_topic, Odometry, gtCallback)
rospy.spin()


#while not rospy.is_shutdown():
    
    #try:
        #(trans,rot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #continue
    #time =  rospy.Time.now()
    ##print(trans)
    
    
    #path.header=std_msgs.msg.Header()
    #path.header.frame_id='odom'
    #path.header.stamp= time
    
    #pose = PoseStamped()
    #pose.header=std_msgs.msg.Header()
    #pose.header.frame_id='odom'
    #pose.header.stamp= time

    #pose_tmp=getPose(trans,rot)
    #pose_tmp.position.x=pose_tmp.position.x-initial_trans[0]
    #pose_tmp.position.y=pose_tmp.position.y-initial_trans[1]
    #pose_tmp.position.z=pose_tmp.position.z-initial_trans[2]
    ##initial_pose
    ##pose.pose=rotations.transform(initial_pose,pose_tmp)
    #pose.pose=pose_tmp
    
    #path.poses.append(pose)    
    #path_pub.publish(path)
