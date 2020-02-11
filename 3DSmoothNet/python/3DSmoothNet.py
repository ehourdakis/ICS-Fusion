import socket
import sys
import os
import numpy as np

import tensorflow as tf
from core import config
from core import network

import copy
import subprocess
from open3d import *
import struct
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

sizeOfInt = 4
sizeOfFloat = 4
server_address = '/tmp/3dsmoothnet'

descr = None
prevDescr = None
keyVert = None
prevKeyVert = None

prevFrame = None
frame = 40


def receiveLrf(conn):
    data = connection.recv(2*sizeOfInt)
    if not data:
        return None, None, None
    keyptsSizeBinary = data[0:sizeOfInt]
    counterVoxelBinary = data[sizeOfInt:2*sizeOfInt]
    
    keyptsSize = int.from_bytes(keyptsSizeBinary,byteorder='little', signed=True)
    counterVoxel = int.from_bytes(counterVoxelBinary,byteorder='little', signed=True)
    ultimate_buffer = bytearray()
    for i in range(0,keyptsSize):        
        receiving_buffer = connection.recv(sizeOfFloat*counterVoxel)        
        ultimate_buffer = ultimate_buffer + receiving_buffer
    dt = np.dtype(np.float32)
    dt = dt.newbyteorder('<')
    x = np.frombuffer(ultimate_buffer, dtype=dt)
    x = x.reshape(keyptsSize,counterVoxel)
    return keyptsSize, counterVoxel, x

def do_execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
    result = registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, reference_desc, target_desc,
            distance_threshold,
            registration.TransformationEstimationPointToPoint(False), 4,
            [registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            registration.RANSACConvergenceCriteria(4000000, 500))
    #print(result.fitness)
    #print(result.inlier_rmse)
    #print(result.correspondence_set)
    #print(result.correspondence_set[0])    
    return result 

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    visualization.draw_geometries([source_temp, target_temp])

def receiveKeyVertex(connection, size):
    ultimate_buffer = bytearray()
    totalSize = 3*size*sizeOfFloat;
    while len(ultimate_buffer) < totalSize:
        receiving_buffer = connection.recv( min(1024, totalSize - len(ultimate_buffer)) )
        ultimate_buffer = ultimate_buffer + receiving_buffer
        
    dt = np.dtype(np.float32)
    dt = dt.newbyteorder('<')
    x = np.frombuffer(ultimate_buffer, dtype=dt)
    x = x.reshape(size,3)
    return x

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
    
    """
    point_cloud_files = [ "./data/ply/f_" + str(prevFrame) + "_vertices.ply","./data/ply/f_" + str(frame) + "_vertices.ply" ]
    reference_pc = read_point_cloud(point_cloud_files[0])
    test_pc = read_point_cloud(point_cloud_files[1])
    draw_registration_result(test_pc,reference_pc,result_ransac.transformation)
    """
    
    corr = np.asarray(result_ransac.correspondence_set, dtype=np.int32)
    return result_ransac.fitness, result_ransac.inlier_rmse, result_ransac.transformation, corr
    
def sendTf(conn, fitness, rmse, tf):
    buff = bytearray()
    buff = buff + struct.pack('f',fitness)
    buff = buff + struct.pack('f',rmse)
    
    for i in range(0,4):
        for j in range(0,4):
            buff = buff + struct.pack('f', tf[i,j])
    
    connection.send(buff)

def sendCorresp(connection, corr):
    buff = bytearray()
    size = len(corr)
    
    buff = buff + struct.pack('i',size)
    for x in corr:
        buff = buff + struct.pack('i',x[0])
        buff = buff + struct.pack('i',x[1])        
    connection.send(buff)

#3DSmoothNet configs
config_arguments, unparsed_arguments = config.get_config()
smooth_net = network.NetworkBuilder(config_arguments)


# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise 
# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
# Bind the socket to the port
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection    
    connection, client_address = sock.accept()
    #global smooth_net
    try:
        while True:
            keyptsSize, counterVoxel, lrf = receiveLrf(connection)
            if  lrf is None:
                continue
            keyVert = receiveKeyVertex(connection, keyptsSize)
            if  keyVert  is None:
                continue

            prevDescr = descr
            descr = smooth_net.test(lrf)
            
            if prevKeyVert is not None:
                #fitness, rmse, tr, corr = execute_global_registration(prevKeyVert, keyVert, prevDescr, descr)
                fitness, rmse, tr, corr = execute_global_registration(keyVert, prevKeyVert, descr, prevDescr)
                sendTf(connection, fitness, rmse, tr)
                sendCorresp(connection, corr)
            else:
                fitness = -1.0
                rmse = -1.0
                tr = np.zeros( (4,4) )
                sendTf(connection, fitness, rmse, tr)
            
            prevKeyVert = keyVert
            
            prevFrame = frame
            frame = frame + 40            
            
            
    finally:
        # Clean up the connection
        connection.close()
        