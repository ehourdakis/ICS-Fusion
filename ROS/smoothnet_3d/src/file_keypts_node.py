#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import Image

import smoothnet_3d.msg

from random import sample
import sys

k = int(sys.argv[1])

rospy.init_node('file_keypts')

client = actionlib.SimpleActionClient('smoothnet_3d', smoothnet_3d.msg.SmoothNet3dAction)
client.wait_for_server()

ply_file = rospy.get_param("~ply_file")
keypts_file = rospy.get_param("~keypts_file")

f = open(ply_file, "r")
size = 0
s_x = []
s_y = []
s_z = []

while True:
    line = f.readline()
    if not line: 
        #fix me
        f.close()
        exit(1)
    words = line.split()        
    if words[0] == "end_header":
        break
    if words[0] == "element" and words[1] == "vertex":
        size = int(words[2])

goal = smoothnet_3d.msg.SmoothNet3dGoal()
for i in range(0,size):
    line = f.readline()
    if not line:
        #fix me
        f.close()
        exit(1)
    words = line.split()
    goal.vert_x.append( float(words[0]) )
    goal.vert_y.append( float(words[1]) )
    goal.vert_z.append( float(words[2]) )

f.close()

goal.prev=0;
goal.seq=1


f = open(keypts_file, "r")

while True:
    line = f.readline()
    if line:    
        #print(line)
        #print(int(line))
        goal.pts.append( int(line) )
    else:
        break
f.close()

client.send_goal(goal)
client.wait_for_result()
results = client.get_result() 

print(results)
