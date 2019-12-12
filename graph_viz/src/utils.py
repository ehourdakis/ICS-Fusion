#!/usr/bin/env python
import rospy
from  geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3 
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA

nodes = dict()
lineId = 1000

def fromVisionCord(pose):
    ret = Pose()
    ret.position.x = pose.position.z
    ret.position.y = -pose.position.x
    ret.position.z = -pose.position.y
    return ret

def toVisionCord(pose):
    ret = Pose()
    ret.position.x = -pose.position.y
    ret.position.y = -pose.position.z
    ret.position.z = pose.position.x
    return ret

def poseMarker(id, pose, rgb):
    global nodes
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 1.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.pose = pose
    marker.id = id    
    nodes[id] = marker
    return marker

def lineMarker(id1,id2):
    global lineId, nodes
 
    p1 = nodes[id1].pose.position
    p2 = nodes[id2].pose.position
    line_color = ColorRGBA()       # a nice color for my line (royalblue)
    line_color.r = 0.7 * id1
    line_color.g = 0.411765
    line_color.b = 0.882353
    line_color.a = 1.0
   
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.type = Marker.LINE_STRIP
    marker.ns = 'Testline'
    marker.action = 0
    marker.scale.x = 0.01        
    marker.points.append(p1)
    marker.points.append(p2)
    marker.colors.append(line_color)
    marker.colors.append(line_color)
    marker.id = lineId
    lineId = lineId + 1
    return marker

def pointMarker(id, position,rgb):
    global nodes
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 1.5
    marker.scale.y = 1.5
    marker.scale.z = 1.5
    marker.color.a = 1.0
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.pose.position = position
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.id = id    
    nodes[id] = marker
    id = id + 1
    return marker
