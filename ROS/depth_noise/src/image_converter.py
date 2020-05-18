#!/usr/bin/env python

import rospy
import std_msgs

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

rospy.init_node('image_converter', anonymous=True)
in_topic = None
out_topic = '/camera/rgb/image_converted'
bridge = CvBridge()
in_type = ''
out_type = ''

def callback(data):
    global image_pub, bridge, prev_image, prev_stamp
    try:
        #cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
        #image_pub.publish(bridge.cv2_to_imgmsg(cv_image, out_type))
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding=out_type)
    except CvBridgeError as e:
        print(e)
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
    #print(data.header.stamp)
    
    try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, out_type))
    except CvBridgeError as e:
      print(e)    
    
        
try:
    in_topic = rospy.get_param("~in_topic")
    in_type = rospy.get_param("~in_type")  
    out_type = rospy.get_param("~out_type")  
except Exception as  e:
    print("could not get parameters")

image_sub = rospy.Subscriber(in_topic,Image,callback)
image_pub = rospy.Publisher(out_topic,Image,queue_size=1000)

rospy.spin()