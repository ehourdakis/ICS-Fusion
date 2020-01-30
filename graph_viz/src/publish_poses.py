#!/usr/bin/env python
import rospy
import sys
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray
from  geometry_msgs.msg import Pose

import math
import tf
import utils

scale = float(sys.argv[1])
origin = [4, 4, 4]
rgb=[0,0,0]


topic = "/poses"
fileName = ""

def readFile(fileName):
    f = open(fileName, "r")
    poses = []
    for line in f:
        pose = Pose()
        
        pos_orient = line.split(' ')
        pos = pos_orient[0]
        orient = pos_orient[1]
        
        position = pos.split(',')
        
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        
        orientation = orient.split(',')
        yaw = float(orientation[0])        
        pitch = float(orientation[1])
        roll = float(orientation[2])
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'ryxz')    
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        poses.append(pose)
        
    f.close()
    return poses

def addMarkers(poses, rgb):
    poseId = 0
    markerArray = MarkerArray()
    #rgb = [1, 0, 0]
    for pose in poses: 
        pose.position.x = pose.position.x * scale - origin[0] * scale
        pose.position.y = pose.position.y * scale - origin[1] * scale
        pose.position.z = pose.position.z * scale - origin[2] * scale
        
        marker = utils.poseMarker(poseId,pose,rgb)
        poseId = poseId + 1
        markerArray.markers.append(marker)
    
    for x in range(1,poseId):
        prev = x - 1
        marker = utils.lineMarker(prev, x)
        markerArray.markers.append(marker)
    return markerArray

rospy.init_node('publish_poses', anonymous=True)
try:
    fileName = rospy.get_param("~file_name")
    rbgStr =rospy.get_param("~rgb")
    rgbList = rbgStr.split(',')
    rgb[0] = int(rgbList[0])
    rgb[1] = int(rgbList[1])
    rgb[2] = int(rgbList[2])
    
    originStr = rospy.get_param("~origin")
    originList = originStr.split(',')
    origin[0] = int(originList[0])
    origin[1] = int(originList[1])
    origin[2] = int(originList[2])
except Exception as  e:
    print("could not get parameters")
    exit(1)

poses = readFile(fileName)
markerArray = addMarkers(poses, rgb)

try:
    pub = rospy.Publisher(topic, MarkerArray, queue_size=10)    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(markerArray)
        rate.sleep()
except rospy.ROSInterruptException:
    pass
 
