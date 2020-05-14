#!/usr/bin/env python

import rospy
import rostopic
import std_msgs
import roslib
#from nav_msgs.msg import Path
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseStamped, Pose

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

prev_time = None
prev_seq = None
def callback(data):
    global prev_time, prev_seq
    h = data.header
    #print(h)
    curr_time = h.stamp
    curr_seq = h.seq
    
    if prev_time is not None:
        time = curr_time - prev_time
        seq = curr_seq - prev_seq                
        print(str(curr_time)+' '+str(curr_seq) )
        #print(seq)
    
    prev_time = curr_time
    prev_seq = curr_seq
    

rospy.init_node('hz', anonymous=True)
topic = '/camera/depth/image_raw'

data_type = rostopic.get_topic_type(topic, blocking=False)[0]
if data_type:
      data_class = roslib.message.get_message_class(data_type)
      print(data_class)
      
topic_sub = rospy.Subscriber(topic,data_class,callback)
rospy.spin()



#bridge = CvBridge()
#std = 0.1

#lastDepth = None
#lastRgb = None

#def depthCallback(data):
    #global lastDepth
    #lastDepth = data
    ##print('depth')

#def rgbCallback(data):
    #global lastRgb
    #lastRgb = data  
    ##print('rgb')
        
#try:
    #depth_in_topic = rospy.get_param("~depth_in_topic")
    #rgb_in_topic = rospy.get_param("~rgb_in_topic")
    #rate = rospy.get_param("~rate")
#except Exception as  e:
    #print("could not get parameters")

#depth_out_topic = '/camera/depth/image_sync'
#rgb_out_topic = '/camera/rgb/image_sync'

#depth_pub = rospy.Publisher(depth_out_topic,Image,queue_size=1000)
#rgb_pub = rospy.Publisher(rgb_out_topic,Image,queue_size=1000)

#depth_sub = rospy.Subscriber(depth_in_topic,Image,depthCallback)
#rgb_sub = rospy.Subscriber(rgb_in_topic,Image,rgbCallback)

#r = rospy.Rate(1)
#i = 0
#while not rospy.is_shutdown():
    #r.sleep()
    #if (lastDepth is not None) and (lastRgb is not None):
        #depth_pub.publish(lastDepth)
        #depth_pub.publish(lastRgb)
        
        #lastDepth = None
        #lastRgb = None
        ##print('edo')
    #print(i)
    #i = i + 1
    
    
#rospy.spin()