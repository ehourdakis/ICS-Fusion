import tf
from geometry_msgs.msg import PoseWithCovarianceStamped ,PoseStamped, Pose2D, Pose
import numpy as np

T_B_P = np.array([ [0,-1,  0 ,0],
           [0, 0, -1, 0],
           [1, 0,  0, 0],
           [0, 0,  0, 1] ])
invT_B_P = np.linalg.inv(T_B_P)

def fromVisionCordinates(M):
    tmp = np.dot(invT_B_P,M)
    return np.dot(tmp,T_B_P)
    

def toVisionCordinates(M):
    tmp = np.dot(T_B_P,M)
    tmp = np.dot(tmp,invT_B_P)    
    return tmp
    
def pose_to_pose2D(pose):
    pose2d = Pose2D() 
    q=[0,0,0,0]
    q[0]=pose.orientation.x
    q[1]=pose.orientation.y
    q[2]=pose.orientation.z
    q[3]=pose.orientation.w
    euler = tf.transformations.euler_from_quaternion(q)        
    pose2d.x=pose.position.x
    pose2d.y=pose.position.y
    pose2d.theta=euler[2]    
    return pose2d 

def pose2D_to_pose(pose2d):
    pose=Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, pose2d.theta);
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def homogeneous(pose):
    if isinstance(pose, Pose2D):
        p = pose2D_to_pose(pose)
    else:
        p = pose

    rot = [ p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    tran = [ p.position.x, p.position.y, p.position.z]
    R = tf.transformations.quaternion_matrix(rot)
    T = tf.transformations.translation_matrix( tran )
    M = tf.transformations.concatenate_matrices(T, R)
    return M

def poseFromHomo(M):
    ret = Pose()
    R = tf.transformations.quaternion_from_matrix(M)
    ret.position.x = M[0][3]
    ret.position.y = M[1][3]
    ret.position.z = M[2][3]
    ret.orientation.x = R[0]
    ret.orientation.y = R[1]
    ret.orientation.z = R[2]
    ret.orientation.w = R[3]
    return ret    
    
def transform(q1, q2):
    M1 = homogeneous(q1)
    M2 = homogeneous(q2)    
    M = np.dot(M2,M1)
    return poseFromHomo(M)

def quaternion_to_rotation(pose, transform):
    q=[0,0,0,0]
    q[0]=transform.orientation.x
    q[1]=transform.orientation.y
    q[2]=transform.orientation.z
    q[3]=transform.orientation.w
    a = tf.transformations.quaternion_matrix(q)
    T = tf.transformations.translation_matrix( [ transform.position.x, transform.position.y, transform.position.z ] )
    M = tf.transformations.concatenate_matrices(T, a)
    q2 = [ pose.position.x, pose.position.y, pose.position.z, 1]
    b = np.dot(M, q2)
    return b

