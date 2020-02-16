// #include"smoothnetcore.h"
#include<vector>
#include"lrfTypes.h"

extern "C" 
{

void test(float *f,int x,int y);

void calculateLRF(int num_voxels, 
                  float sm3d_radius,
                  float smoothing_factor,
                  int *pts,
                  int pts_size,
                  float *vert_x,
                  float *vert_y,
                  float *vert_z,
                  int vert_size,
                  float *lfr);
}
