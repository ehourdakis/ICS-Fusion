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
                                #ndpointer(ctypes.c_int, flags="C_CONTIGUOUS"),
                                ctypes.POINTER(ctypes.c_int),
                                ctypes.c_int,
                                #ndpointer(ctypes.c_float, flags="C_CONTIGUOUS"),
                                #ndpointer(ctypes.c_float, flags="C_CONTIGUOUS"),
                                #ndpointer(ctypes.c_float, flags="C_CONTIGUOUS"),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.c_int,
                                ndpointer(ctypes.c_float, flags="C_CONTIGUOUS") ]
        
        
        
        #self._smoothing_kernel_width = 1.75
        self._smoothing_kernel_width = smoothing_kernel_width
        self._num_voxels = num_voxels
        #lrf_radius = sqrt(3)*params.sm3d_radius; 
        #self._lrf_radius = sm3d_radius 
        self._radius = sm3d_radius
        self._smoothing_factor = self._smoothing_kernel_width * (sm3d_radius / num_voxels) # Equals half a voxel size so that 3X is 1.5 voxel
        
        counter_voxel = num_voxels*num_voxels*num_voxels
        self.max_size = max_size
        self.lrf_size = counter_voxel * max_size
        max_size = 100
        #self.lrf = (c_float * self.lrf_size) ()
        #self._n = np.empty((counter_voxel,max_size),dtype=np.float32 )
        n = np.empty((max_size,counter_voxel),dtype=np.float32 )
        self._n = np.ascontiguousarray(n, dtype=np.float32)
        #self._n[0,0] = 10
        
        

    def test(self):
        global lib

        c = self._num_voxels * self._num_voxels * self._num_voxels
        nn = np.ones((c,100),dtype=np.float32 )
        self._n2 = np.ascontiguousarray(nn, dtype=np.float32)
        self._n2[0,0] = 10
        #print(arr.data.c_contiguous)
        lib.test(self._n2.ctypes.data_as(ctypes.POINTER(ctypes.c_float) ),
             c_int(100),
             c_int(c) )
        
        print(self._n2)
        print(self._n.data.c_contiguous)
        return
        #n = np.ctypeslib.as_array(self.lrf)        

    def getLrf(self):
        #n = np.ctypeslib.as_array(self.lrf)
        #n = self._n
        return self._n
        
    def calculateLrf(self,goal):
        global lib
        pts_arr = (c_int * len(goal.pts))(*goal.pts)
        vert_x_arr = (c_float * len(goal.vert_x))(*goal.vert_x)
        vert_y_arr = (c_float * len(goal.vert_y))(*goal.vert_y)
        vert_z_arr = (c_float * len(goal.vert_z))(*goal.vert_z)
 
        #print(goal.vert_x)
        
        #print(type(self.lrf) )
        #arr = c_float * self.lrf_size
        
        
        
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
        
        
        """
        lib.calculateLRF(c_int(self._num_voxels),
                 c_float(self._radius),
                 c_float(self._smoothing_factor),
                 pts_arr,
                 c_int( len(goal.pts) ),
                 vert_x_arr,
                 vert_y_arr,
                 vert_z_arr,
                 c_int( len(goal.vert_x) ),
                 #self.lrf
                 self._n.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
                 )
        """
        print("done")
        #print(self.lrf[9])
