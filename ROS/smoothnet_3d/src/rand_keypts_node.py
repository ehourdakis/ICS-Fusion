#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import Image

import smoothnet_3d.msg

from random import sample
import sys

k = int(sys.argv[1])

#def image_callback(image):
    #print('test')
    
    
rospy.init_node('rand_keypts')
image_topic = rospy.get_param("~depth_topic")

image = rospy.wait_for_message(image_topic, Image)


size = image.height*image.width

xlist = range(size)
r=sample(xlist,k)

print(size)


client = actionlib.SimpleActionClient('smoothnet_3d', smoothnet_3d.msg.SmoothNet3dAction)
client.wait_for_server()

goal = smoothnet_3d.msg.SmoothNet3dGoal()
goal.image_seq = image.header.seq
goal.image_time = image.header.stamp
goal.pts=r
print(goal)

client.send_goal(goal)
client.wait_for_result()
results = client.get_result() 

print(results)


#rospy.Subscriber(image_topic, Image, image_callback)



#keypoints_dir = "./data/keypts/"
#descr_dir = "./data/descr/"

#infile = keypoints_dir +"f_" + sys.argv[1] + "_keypoints.txt"
#outfile = keypoints_dir +"f_" + sys.argv[1] + "_keypoints2.txt"

#inDescr = descr_dir +"frame" + sys.argv[1] + ".csv"
#outDescr = descr_dir +"frame" + sys.argv[1] + "_2.csv"

#f = open(infile, "r",errors='replace')
#oldKeypts = f.readlines()
#f.close()

#f = open(inDescr, "r",errors='replace')
#odlDescr = f.readlines()
#f.close()


#size=len(oldKeypts)
#print( "Choosing %d keypoints from %d vertexes." % (k, size ) )

#xlist = range(size)
#r=sample(xlist,k)

#f = open(outfile,"w")
#descrFile = open(outDescr, "w")
#for i in r:
    #f.write( oldKeypts[i] + "\n" )
    #descrFile.write( odlDescr[i] + "\n" )
#f.close() 
#descrFile.close()