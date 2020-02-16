#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import actionlib
import smoothnet_3d.msg
from collections import deque
import numpy as np
import os
import tensorflow as tf
import copy
from open3d import *

file_dir=os.path.dirname(__file__)
sys.path.append( os.path.join(file_dir, "3DSmoothNet/src/") )
import LrfWrapper
from core import config
from core import network



lib_name = 'libsmoothnet_3d_lrf.so'
lib_path = os.path.join(file_dir,'3DSmoothNet/build/',lib_name)


server = None
lrf = None
smooth_net = None
prevDescr = None
descr = None
keyVert = None
prevKeyVert = None

vert = None
prevVert = None

draw_registration = False

def do_execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
    result = registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, reference_desc, target_desc,
            distance_threshold,
            registration.TransformationEstimationPointToPoint(False), 4,
            [registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            registration.RANSACConvergenceCriteria(4000000, 500)) 
    return result 

def execute_global_registration(reference_pc_keypoints, test_pc_keypoints, reference_desc, test_desc):    
    # Save ad open3d point clouds
    ref_key = geometry.PointCloud()
    ref_key.points = utility.Vector3dVector(reference_pc_keypoints)

    test_key = geometry.PointCloud()
    test_key.points = utility.Vector3dVector(test_pc_keypoints)

    # Save as open3d feature 
    ref = open3d.registration.Feature()
    ref.data = reference_desc.T

    test = open3d.registration.Feature()
    test.data = test_desc.T
    result_ransac = do_execute_global_registration(ref_key, test_key,ref, test, 0.05)

    corr = np.asarray(result_ransac.correspondence_set, dtype=np.int32)
    return result_ransac.fitness, result_ransac.inlier_rmse, result_ransac.transformation, corr

def extractKeyVert(vert_x, vert_y, vert_z, pts):
    size = len(pts)
    arr = np.empty( (size, 3) )
    
    for i in range(0,size):
        pt = pts[i]
        x = vert_x[pt]
        y = vert_y[pt]
        z = vert_z[pt]
        
        arr[i,0] = x
        arr[i,1] = y
        arr[i,2] = z
        
    return arr
    
def extractVert(vert_x, vert_y, vert_z):    
    size = len(vert_x)
    arr = np.empty( (size, 3) )
    for i in range(0, size):
        x = vert_x[i]
        y = vert_y[i]
        z = vert_z[i]
        
        arr[i,0] = x
        arr[i,1] = y
        arr[i,2] = z
    return arr
        
def do_draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    visualization.draw_geometries([source_temp, target_temp])
    
def draw_registration_result(source, target, transformation):
    
    ref_cloud = geometry.PointCloud()
    ref_cloud.points = utility.Vector3dVector(source)
    
    target_cloud = geometry.PointCloud()
    target_cloud.points = utility.Vector3dVector(target)
    
    do_draw_registration_result(ref_cloud, target_cloud, transformation)
    
    
def execute_cb(goal):
    global lrf, server, smooth_net, prevDescr, descr, keyVert, prevKeyVert, vert, prevVert

    print("new goal")
    
    if len(goal.pts) < 10:
        server.set_aborted(None, 'Too few keypoints')
        return
    print("calculateLrf")
    lrf.calculateLrf(goal.vert_x,
                     goal.vert_y,
                     goal.vert_z,
                     goal.pts )
    
    print("lrf calculated")
    
    data = lrf.getLrf()
    prevDescr = descr
    prevKeyVert = keyVert
    
    if draw_registration:
        prevVert = vert
        vert = extractVert(goal.vert_x, goal.vert_y, goal.vert_z)
    
    descr = smooth_net.test(data)
    
    print("descriptors found")
    
    keyVert = extractKeyVert(goal.vert_x,
                   goal.vert_y,
                   goal.vert_z,
                   goal.pts )
    
    result = smoothnet_3d.msg.SmoothNet3dResult()
    
    if prevKeyVert is not None:
        fitness, rmse, tr, corr = execute_global_registration(prevKeyVert, keyVert, prevDescr, descr)
        print("tf found")
        result.fitness = fitness
        result.rmse = rmse
        for i in range(0,4):
            for j in range(0,4):
                val = tr[i,j]
                result.tf.append(val)
        
        for c in corr:
            result.source_corr.append(c[0])
            result.target_corr.append(c[1])
        
        if draw_registration:
            draw_registration_result(prevVert, vert, tr)
    else:
        result.fitness = -1.0
        result.rmse = -1.0

    print("done")
        

    server.set_succeeded(result)



rospy.init_node('smoothnet_3d')

num_voxels = rospy.get_param("~num_voxels")
sm3d_radius = rospy.get_param("~radius")
kepts_size = rospy.get_param("~kepts_size")
smoothing_kernel_width = rospy.get_param("~smoothing_kernel_width")
draw_registration = rospy.get_param("~draw_registration")

lrf = LrfWrapper.Lrf(lib_path,
                     num_voxels,
                     sm3d_radius,
                     smoothing_kernel_width,
                     kepts_size )


#3DSmoothNet configs
config_arguments, unparsed_arguments = config.get_config()
print(config_arguments)
config_arguments.saved_model_dir =  os.path.join(file_dir, "3DSmoothNet/models/")
config_arguments.training_data_folder =  os.path.join(file_dir, "3DSmoothNet/train/")
smooth_net = network.NetworkBuilder(config_arguments)

server = actionlib.SimpleActionServer('smoothnet_3d', smoothnet_3d.msg.SmoothNet3dAction, execute_cb=execute_cb, auto_start = False)
server.start()

rospy.spin()
