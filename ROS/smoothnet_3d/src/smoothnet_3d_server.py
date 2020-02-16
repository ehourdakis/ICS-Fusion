#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import actionlib
import smoothnet_3d.msg
from collections import deque
import LrfWrapper
import numpy as np
lib_path = '/home/tavu/workspace/ICS-Fusion/3DSmoothNet/build/'
lib_name = 'libsmoothnet_3d_lrf.so'

server = None
lrf = None

def execute_cb(goal):
    global lrf, server
    lrf.calculateLrf(goal)
    #lrf.test()
    data = lrf.getLrf()
    np.savetxt('/tmp/data1.csv', data, delimiter=',',fmt='%1.8f')
    #print(n)
    #lrf.test()
    #print(goal.pts)

    #print(type( bytes(goal.pts)) )
    #print(bytes(goal.pts))
    
    
    server.set_aborted(None)



rospy.init_node('smoothnet_3d')

num_voxels = 16
sm3d_radius = 0.15;
max_size = 500
smoothing_kernel_width = 1.75
lrf = LrfWrapper.Lrf(lib_path+lib_name,
                     num_voxels,
                     sm3d_radius,
                     smoothing_kernel_width,
                     max_size )

server = actionlib.SimpleActionServer('smoothnet_3d', smoothnet_3d.msg.SmoothNet3dAction, execute_cb=execute_cb, auto_start = False)
server.start()

#lib.test()

rospy.spin()
