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
        self._counter_voxel = num_voxels*num_voxels*num_voxels
        self._max_size = max_size
        n = np.empty((self._max_size, self._counter_voxel),dtype=np.float32 )
        self._n = np.ascontiguousarray(n, dtype=np.float32)        

    def getLrf(self):
        return self._n
        
    def calculateLrf(self, vert_x, vert_y, vert_z, pts):
        global lib
        pts_arr = (c_int * len(pts))(*pts)
        vert_x_arr = (c_float * len(vert_x))(*vert_x)
        vert_y_arr = (c_float * len(vert_y))(*vert_y)
        vert_z_arr = (c_float * len(vert_z))(*vert_z)
        
        if len(pts) != self._max_size:
            self._max_size = len(pts)
            n = np.empty( (self._max_size, self._counter_voxel), dtype=np.float32 )
            self._n = np.ascontiguousarray(n, dtype=np.float32)
            print()
 
        self._calc(c_int(self._num_voxels),
                 c_float(self._radius),
                 c_float(self._smoothing_factor),
                 pts_arr,
                 c_int( len(pts) ),
                 vert_x_arr,
                 vert_y_arr,
                 vert_z_arr,
                 c_int( len(vert_x) ),
                 #self.lrf
                 self._n)
