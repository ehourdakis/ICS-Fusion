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
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

sizeOfInt = 4
sizeOfFloat = 4
server_address = '/tmp/3dsmoothnet'

descr = None
oldDescr = None

prevFrame = None
frame = 40


def receiveLrf(conn):
    data = connection.recv(2*sizeOfInt)
    if not data:
        return None
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
    return x

def execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, reference_desc, target_desc,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    #print(result.fitness)
    return result 

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


def registration():    
    global frame,prevFrame
    
    if prevFrame is None:
        return
    
    point_cloud_files = [ "./data/ply/f_" + str(prevFrame) + "_vertices.ply",
                          "./data/ply/f_" + str(frame) + "_vertices.ply" ]
        
    keypoints_files = [ "./data/keypts/f_" + str(prevFrame) + "_keypoints.txt",
                       "./data/keypts/f_" + str(frame) + "_keypoints.txt" ]
    
    outfile_name="./data/transformations/frame" + str(frame) + ".txt"

    print(point_cloud_files)
    print(keypoints_files)
    
    
    reference_pc = read_point_cloud(point_cloud_files[0])
    test_pc = read_point_cloud(point_cloud_files[1])

    indices_ref = np.genfromtxt(keypoints_files[0])
    indices_test = np.genfromtxt(keypoints_files[1])

    reference_pc_keypoints = np.asarray(reference_pc.points)[indices_ref.astype(int),:]
    test_pc_keypoints = np.asarray(test_pc.points)[indices_test.astype(int),:]

    reference_desc = oldDescr    
    test_desc = descr;
    
    # Save ad open3d point clouds
    ref_key = PointCloud()
    ref_key.points = Vector3dVector(reference_pc_keypoints)

    test_key = PointCloud()
    test_key.points = Vector3dVector(test_pc_keypoints)

    # Save as open3d feature 
    ref = open3d.registration.Feature()
    ref.data = reference_desc.T

    test = open3d.registration.Feature()
    test.data = test_desc.T
    result_ransac = execute_global_registration(ref_key, test_key,ref, test, 0.05)
    draw_registration_result(reference_pc, test_pc,result_ransac.transformation)

    trans=result_ransac.transformation
    f = open(outfile_name,"w")

    f.write( str(result_ransac.fitness) + '\n' )
    f.write( str(result_ransac.inlier_rmse) + '\n' )
    
    for x in range(0,4):
        for y in range(0,4):
            f.write( str(trans[x,y])+" " )
        f.write('\n')
    f.close()

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
            lrf = receiveLrf(connection)            
            if  lrf is None:
                continue
            
            #config_arguments, unparsed_arguments = config.get_config()
            evaluate_input_file = "./data/sdv/frame" + str(frame) + ".sdv"
            evaluate_output_file = "./data/descr/frame" + str(frame) + ".csv"
                 
            oldDescr = descr
            descr = smooth_net.test(lrf,evaluate_input_file,evaluate_output_file)
            
            registration()
            
            status = 10
            print(status)
            statusBytes = status.to_bytes(1,byteorder='little')
            connection.send(statusBytes)
            
            
            
            prevFrame = frame
            frame = frame + 40            
            
            

    finally:
        # Clean up the connection
        connection.close()
        