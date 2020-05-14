#!/usr/bin/env python 
import rospy
import sys
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Float64

image_topic="/camera/rgb/image_raw"

def variance_of_laplacian(image):
    # compute the Laplacian of the image and then return the focus
    # measure, which is simply the variance of the Laplacian
    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    laplacian1 = laplacian/laplacian.max()
    return laplacian1.var()

def detect_blur(image):
    thr = 1000
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    
    if fm < thr:
        print("Blurry")
    
    return fm

def callback(data):
    global pub
    bridge = CvBridge()
    print('here')
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    
    b = detect_blur(cv_image)
    b = b * 100
    print(b)
    msg = Float64()
    msg.data = b
    pub.publish(msg)

    
    #cv2.imshow("Image window", cv_image)    
    #cv2.waitKey(3)

rospy.init_node('detect_blur')

pub = rospy.Publisher('blur', Float64)
image_sub = rospy.Subscriber(image_topic,Image,callback)
rospy.spin()
