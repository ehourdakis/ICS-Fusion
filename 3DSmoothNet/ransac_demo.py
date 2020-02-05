import sys
import os

#import tensorflow as tf
import copy
import numpy as np
import subprocess
from open3d import *
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

#import tensorflow.python.util.deprecation as deprecation
#deprecation._PRINT_DEPRECATION_WARNINGS = False

#print(tf.python.util)
#tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

point_cloud_files = [sys.argv[1], sys.argv[2]]
descr_files = [sys.argv[3], sys.argv[4]]
keypoints_files = [sys.argv[5], sys.argv[6]]
outfile_name = sys.argv[7]
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


def execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, reference_desc, target_desc,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    print(result.fitness)
    return result 


reference_pc = read_point_cloud(point_cloud_files[0])
test_pc = read_point_cloud(point_cloud_files[1])

#print(reference_pc.size())
#print(test_pc.size())

indices_ref = np.genfromtxt(keypoints_files[0])
indices_test = np.genfromtxt(keypoints_files[1])

reference_pc_keypoints = np.asarray(reference_pc.points)[indices_ref.astype(int),:]
test_pc_keypoints = np.asarray(test_pc.points)[indices_test.astype(int),:]

reference_desc = np.genfromtxt( descr_files[0],delimiter=",")
test_desc = np.genfromtxt(descr_files[1],delimiter=",")
#reference_desc = np.load( descr_files[0] )
#reference_desc = reference_desc['data']

#test_desc = np.load( descr_files[1] )
#test_desc = test_desc['data']

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

#print(len(ref_key))
#print(len(test_key))
result_ransac = execute_global_registration(ref_key, test_key,ref, test, 0.05)

#print(result_ransac
#test_desc = test_desc['data']

# First plot the original state of the point clouds

#print(result_ransac)
#print(result_ransac.transformation)


#draw_registration_result(reference_pc, test_pc, np.identity(4))

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

