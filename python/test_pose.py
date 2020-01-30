import sys
import os
import math
import numpy as np
import copy
import subprocess
from open3d import *
#from scipy.spatial.transform import Rotation as R

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
      
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])    
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x )) 
    return R

def homo(trans, rot):
    R = eulerAnglesToRotationMatrix(rot)    
    ret = np.identity(4)
    
    for i in range(0,3):
        for j in range(0,3):
            ret[i,j] = R[i,j]
    
    ret[0,3] = trans[0]    
    ret[1,3] = trans[1]
    ret[2,3] = trans[2]
    return ret
    
def readPoseFile(filename):    
    poseFile = open(filename, "r")
    
    """
    ret = np.zeros( (4,4) )

    for i in range(0,4):
        line = poseFile.readline()
        values = line.split()
        print(values)
        for j in range(0,4):
            ret[i,j] = float(values[j])
    ret = np.linalg.inv(ret)
    return ret
    """    
    line = poseFile.readline()
    [transStr,rotStr] = line.split()
    transArray = transStr.split(',')    
    trans = [ float(transArray[0]), float(transArray[1]), float(transArray[2]) ]    
    
    rotArray = rotStr.split(',')
    rot = [ float(rotArray[2]), float(rotArray[1]) ,float(rotArray[0]) ]
    
    print(trans)
    print(rot)
    mat = homo(trans,rot)

    return np.linalg.inv(mat)
        
    

frame1 = int(sys.argv[1])
frame2 = int(sys.argv[2])

point_cloud_dir = "./data/ply/"
poseDir = './data/gt/'
poseFileName = 'f_' + str(frame2) + '_pose'

point_cloud_names = ["f_" + str(frame1) + "_vertices.ply", "f_" + str(frame2) + "_vertices.ply"] 
point_cloud_files = [point_cloud_dir + point_cloud_names[0], point_cloud_dir + point_cloud_names[1]]


delta=readPoseFile(poseDir + poseFileName)
#print(delta)

reference_pc = read_point_cloud(point_cloud_files[0])
test_pc = read_point_cloud(point_cloud_files[1])

#draw_registration_result(reference_pc, test_pc, np.identity(4))

draw_registration_result(reference_pc, test_pc, delta)










