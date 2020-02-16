#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import Image
import smoothnet_3d.msg
from random import sample
import sys
import numpy as np

def getGoal(ply_file, k):
    size = 0
    s_x = []
    s_y = []
    s_z = []
    f = open(ply_file, "r")
    while True:
        line = f.readline()
        if not line: 
            #fix me
            print('error reading file')
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
            print('error reading vertices')
            f.close()
            exit(1)
        words = line.split()
        goal.vert_x.append( float(words[0]) )
        goal.vert_y.append( float(words[1]) )
        goal.vert_z.append( float(words[2]) )
    f.close()
    
    xlist = range(size)
    r=sample(xlist,k)
    goal.pts.extend(r)
    return goal

k = int(sys.argv[1])
rospy.init_node('file_keypts')
client = actionlib.SimpleActionClient('smoothnet_3d', smoothnet_3d.msg.SmoothNet3dAction)
client.wait_for_server()

ply_file1 = rospy.get_param("~ply_file1")
ply_file2 = rospy.get_param("~ply_file2")

goal1 = getGoal(ply_file1, k)
client.send_goal(goal1)
client.wait_for_result()
results = client.get_result() 
print(results)

print("")

goal2 = getGoal(ply_file2, k)
client.send_goal(goal2)
client.wait_for_result()
results = client.get_result() 
print(results)

tf = np.empty( (4,4) )

for i in range(0,4):
    for j in range(0,4):
        tf[i,j] = results.tf[4*i+j]

print(tf)
