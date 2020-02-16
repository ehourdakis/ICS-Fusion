from ctypes import *
import ctypes
import math
lib = None
import numpy as np
#import tensorflow as tf
from numpy.ctypeslib import ndpointer
class Lrf(object):
    def __init__(self,
                 libPath,
                 num_voxels,
                 sm3d_radius,
                 smoothing_kernel_width,
                 max_size ):
        global lib
        lib = cdll.LoadLibrary(libPath)
        self._calc = lib.calculateLRF
        
        
        self._calc .argtypes = [ctypes.c_int,
                                ctypes.c_float,
                                ctypes.c_float,
                                ctypes.POINTER(ctypes.c_int),
                                ctypes.c_int,
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.c_int,
                                ndpointer(ctypes.c_float, flags="C_CONTIGUOUS") ]

        self._smoothing_kernel_width = smoothing_kernel_width
        self._num_voxels = num_voxels
        self._radius = sm3d_radius
        self._smoothing_factor = self._smoothing_kernel_width * (sm3d_radius / num_voxels) # Equals         
        counter_voxel = num_voxels*num_voxels*num_voxels
        self.max_size = max_size
        self.lrf_size = counter_voxel * max_size
        max_size = 100
        n = np.empty((max_size,counter_voxel),dtype=np.float32 )
        self._n = np.ascontiguousarray(n, dtype=np.float32)        

    def getLrf(self):
        return self._n
        
    def calculateLrf(self,goal):
        global lib
        pts_arr = (c_int * len(goal.pts))(*goal.pts)
        vert_x_arr = (c_float * len(goal.vert_x))(*goal.vert_x)
        vert_y_arr = (c_float * len(goal.vert_y))(*goal.vert_y)
        vert_z_arr = (c_float * len(goal.vert_z))(*goal.vert_z)
 
        self._calc(c_int(self._num_voxels),
                 c_float(self._radius),
                 c_float(self._smoothing_factor),
                 pts_arr,
                 c_int( len(goal.pts) ),
                 vert_x_arr,
                 vert_y_arr,
                 vert_z_arr,
                 c_int( len(goal.vert_x) ),
                 #self.lrf
                 self._n)        
        print("done")
