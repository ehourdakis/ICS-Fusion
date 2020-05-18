#!/usr/bin/env python

import rospy
import std_msgs

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

rospy.init_node('depth_noise', anonymous=True)
in_topic = None
out_topic = '/camera/depth/image_noise'
bridge = CvBridge()
std = 0.1

def callback(data):
    global image_pub, bridge, prev_image, prev_stamp
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
        print(e)

    out_image = np.zeros(cv_image.shape, dtype=cv_image.dtype)    
    cv2.randn(out_image, 0.0, std) 

    
    cv_image = cv_image + out_image
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
    #print(data.header.stamp)
    
    try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "passthrough"))
    except CvBridgeError as e:
      print(e)    
    
        
try:
    in_topic = rospy.get_param("~in_topic")
    std = rospy.get_param("~std")  
except Exception as  e:
    print("could not get parameters")

print(std)
image_sub = rospy.Subscriber(in_topic,Image,callback)
image_pub = rospy.Publisher(out_topic,Image,queue_size=1000)

rospy.spin()