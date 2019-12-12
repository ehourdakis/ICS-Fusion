#!/usr/bin/env python
import rospy
import utils
import sys
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray
from  geometry_msgs.msg import Pose
import rotations as Rot
import math
import tf

startFrame = 0
endFrame = 120

origin = [4, 4, 4]
#origin = [0, 0, 0]
#origin = [4, 4, 4]

scale = float(sys.argv[2])

#scale = 1

lineId = 1000
poseId = 0
nodes = dict()
filePath = '/home/tavu/workspace/slambench2/f_/'
fileName = 'f_%_poses'



markerArray = MarkerArray()

def addLoopClosurePose(pose):
    global poseId, lineId, markerArray
    rgb = [1, 1, 1]    
    
    poseMarker = utils.poseMarker(poseId,pose,rgb)
    lineMarker = utils.lineMarker(0, poseId)
    markerArray.markers.append(poseMarker)
    markerArray.markers.append(lineMarker)
    poseId = poseId + 1


def addMarkers(poses):
    global poseId, lineId, markerArray
    rgb = [0, 0, 1]
    for pose in poses: 
        #pose = utils.fromVisionCord(pose)
        
        #pose.position.y = - pose.position.y
        #pose.position.x = - pose.position.x
        #pose.position.z = - pose.position.z
        
        marker = utils.poseMarker(poseId,pose,rgb)
        poseId = poseId + 1
        markerArray.markers.append(marker)
    
    for x in range(1,poseId):
        prev = x - 1
        marker = utils.lineMarker(prev, x)
        markerArray.markers.append(marker)
    
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
        roll = float(orientation[0])
        pitch = float(orientation[1])
        yaw = float(orientation[2])
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'ryxz')    
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        poses.append(pose)
        
    f.close()
    return poses
    
try:
    pub = rospy.Publisher('graph_poses', MarkerArray, queue_size=10)
    rospy.init_node('kfusion_viz', anonymous=True)    
    rate = rospy.Rate(10) 
    
    lineId = 1000
    poseId = 0

    frame = sys.argv[1]
    fileName = fileName.replace('%',frame) 
        
    poses = readFile(filePath + fileName.replace('%',frame) )
    
    for i in range( len(poses) ):
        pose = poses[i]
        pose.position.x = pose.position.x * scale
        pose.position.y = pose.position.y * scale
        pose.position.z = pose.position.z * scale
        
        #M = Rot.homogeneous(pose)
        #M = Rot.fromVisionCordinates(M)
        #pose = Rot.poseFromHom(M)
        
        pose.position.x = pose.position.x - origin[0] * scale
        pose.position.y = pose.position.y - origin[1] * scale
        pose.position.z = pose.position.z - origin[2] * scale
        
        poses[i] = pose
    
    #addMarkers(poses[0:len(poses)-1])
    addMarkers(poses[0:len(poses)])
    #addLoopClosurePose(poses[-1])
    odom_pub = rospy.Publisher("static_odom", Odometry, queue_size=50)
    while not rospy.is_shutdown():
        pub.publish(markerArray)
        rate.sleep()
except rospy.ROSInterruptException:
    pass
