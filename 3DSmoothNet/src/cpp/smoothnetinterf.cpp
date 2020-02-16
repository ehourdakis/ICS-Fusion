#include"smoothnetinterf.h"
#include <numpy/arrayobject.h>
#include<iostream>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include"smoothnetcore.h"

void test(float *f,int x,int y)
{
//     std::cout<<"s"<<f[1][1]<<std::endl;
    for(int i=0;i<x;i++)
    {
        for(int j=0;j<y;j++)
        {
            int idx=i + j*x;
            std::cout<<"s "<<f[idx]<<std::endl;
            f[idx]=10*i+j;
        }
    }
            
        

}

// void calculateLRF(std::vector<int> evaluation_points,
//              std::vector<Vertex> vertices,
//              int num_voxels,
//              float lrf_radius,
//              float smoothing_factor,
//              float sm3d_radius,
//              float *lfr)
void calculateLRF(int num_voxels, 
          float radius,
          float smoothing_factor,
          int *evaluation_points,
          int pts_size,
          float *vert_x,
          float *vert_y,
          float *vert_z,
          int vert_size,
          float *lrf)

{
#if 1
    //create pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<vert_size;i++)
    {
        pcl::PointXYZ pt(vert_x[i],vert_y[i],vert_z[i]);
        cloud->push_back(pt);
    }
 
    float voxel_step_size = (2 * radius) / num_voxels;
    flann::Matrix<float> voxel_coordinates = initializeGridMatrix(num_voxels, voxel_step_size, voxel_step_size, voxel_step_size);
    
    int counter_voxel = num_voxels*num_voxels*num_voxels;
    
    std::vector<int> indices(cloud->width);
    std::vector<LRF> cloud_lrf(cloud->width);
    std::iota(indices.begin(), indices.end(), 0);
    std::vector <std::vector <int>> nearest_neighbors(cloud->width);
    std::vector <std::vector <int>> nearest_neighbors_smoothing(cloud->width);
    std::vector <std::vector <float>> nearest_neighbors_smoothing_dist(cloud->width);

    // Compute the local reference frame for the interes points
    toldiComputeLRF(cloud,
                    evaluation_points,
                    pts_size,
                    sqrt(3)*radius, 
                    3*smoothing_factor,
                    cloud_lrf,
                    nearest_neighbors,
                    nearest_neighbors_smoothing,
                    nearest_neighbors_smoothing_dist);
    
    // Compute the SDV representation for all the points
    computeLocalDepthFeature(cloud,
                             evaluation_points,
                             pts_size,
                             nearest_neighbors,
                             cloud_lrf,
                             radius,
                             voxel_coordinates,
                             counter_voxel,
                             smoothing_factor,                             
                             lrf);
#endif

}
