#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import actionlib
import smoothnet_3d.msg
from collections import deque

class SmoothNet3DServer(object):
    def __init__(self, name):
        self._images = deque()
        self._as = actionlib.SimpleActionServer(name, smoothnet_3d.msg.SmoothNet3dAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._max_images = 5

    def callSmoothNet3D(self, image, pts):
        print('Success')
        result = smoothnet_3d.msg.SmoothNet3dResult()
        return result
        
        
    def execute_cb(self, goal):
        # helper variables
        print(goal)
        
        for image in self._images:
            if image.header.seq == goal.image_seq:
                _result = self.callSmoothNet3D(image, goal.pts)
                self._as.set_succeeded(_result)  
                return
        self._as.set_aborted(None)
        
    def addImage(self, image):
        self._images.append(image)
        if len(self._images) >= self._max_images:
            self._images.popleft()
        

rospy.init_node('smoothnet_3d')
image_topic = rospy.get_param("~depth_topic")
server = SmoothNet3DServer('smoothnet_3d')
rospy.Subscriber(image_topic, Image, server.addImage)
rospy.spin()