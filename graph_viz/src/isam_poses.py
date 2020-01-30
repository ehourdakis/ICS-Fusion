#!/usr/bin/env python
import rospy
import utils
import sys
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from  geometry_msgs.msg import Pose, Point
import rospy
import math
import tf
import rotations as Rot

origin = [0, 0, 0]
rgb = [0,0,0]
scale = float(sys.argv[1])
lineId = 5000
nodes = dict()
topic = 'isam_poses'

markerArray = MarkerArray()

def publishMarkers():    
    pub.publish(markerArray)
    rate.sleep()

def addNodes(lines,rgb):
    global markerArray
    for line in lines:
        words = line.split()                
        if words[0] == 'Pose3d_Node':
            id = int(words[1])
            pose = Pose()
            
            pose.position.x = float(words[2]) * scale 
            pose.position.y = float(words[3]) * scale 
            pose.position.z = float(words[4]) * scale 
            
            yaw = float(words[5])
            pitch = float(words[6])
            roll = float(words[7])
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'ryxz')
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            #M = Rot.homogeneous(pose)
            #M = Rot.toVisionCordinates(M)
            #pose = Rot.poseFromHom(M)
            
            pose.position.x = pose.position.x - origin[0] * scale
            pose.position.y = pose.position.y - origin[1] * scale
            pose.position.z = pose.position.z - origin[2] * scale

            marker = utils.poseMarker(id, pose, rgb)
            markerArray.markers.append(marker)
        elif words[0] == 'Point3d_Node':
            id = int(words[1])
            p = Point()
            p.x = float(words[2]) * scale
            p.y = float(words[3]) * scale
            p.z = float(words[4]) * scale
            
            p.x = p.x - origin[0] * scale
            p.y = p.y - origin[1] * scale
            p.z = p.z - origin[2] * scale
            
            marker = utils.pointMarker(id, p, rgb)
            markerArray.markers.append(marker)

def addLines(lines):
    global markerArray
    for line in lines:
        words = line.split()                
        if words[0] == 'Pose3d_Pose3d_Factor' or words[0] == 'Pose3d_Point3d_Factor':
             id1 = int(words[1])
             id2 = int(words[2])
             marker = utils.lineMarker(id1,id2)
             markerArray.markers.append(marker)
        
             
def readFile(fileName):
    
    f = open(fileName, "r")
    lines = []
    
    for line in f:
        line = line.replace('(','')
        line = line.replace(')','')
        line = line.replace(';','')
        line = line.replace(',',' ')
        lines.append(line)
    
    f.close()
    addNodes(lines,rgb)
    addLines(lines)
    
rospy.init_node('isam_poses', anonymous=True)
try:
    fileName = rospy.get_param("~file_name")    
    rbgStr =rospy.get_param("~rgb")
    originStr = rospy.get_param("~origin")
    
    rgbList = rbgStr.split(',')    
    rgb[0] = int(rgbList[0])
    rgb[1] = int(rgbList[1])
    rgb[2] = int(rgbList[2])
    
    originList = originStr.split(',')
    origin[0] = int(originList[0])
    origin[1] = int(originList[1])
    origin[2] = int(originList[2])
except Exception as  e:
    print("could not get parameters")
    exit(1)    
    
try:
    pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    readFile(fileName)
    while not rospy.is_shutdown():
        publishMarkers()
        rate.sleep()
except rospy.ROSInterruptException:
    pass
