#include"smoothnet.h"

#include<fstream>
#include<eigen3/Eigen/Dense>
#define KEYPTS_F "/home/tavu/workspace/3DSmoothNet/data/mine/keypts/f_20_keypoints.txt"
//#include <pcl/common//*centroid*/.h>
//#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <pcl/common/centroid.h>
#include"smoothnetcore.h"

#define SDV_DIR "./data/sdv/"
#define CSV_DIR "./data/csv/"
#define OUT_FILE_NAME "./data/sdv/smoothnet.csv"
#define CSV_FILE "./data/csv/smoothnet.csv_3DSmoothNet.txt"
SmoothNet::SmoothNet(IcsFusion *f)
{
    _fusion=f;

    smoothing_kernel_width=1.75;
    num_voxels=16;
    radius=0.150;

    grid_size = num_voxels * num_voxels * num_voxels;
    voxel_step_size = (2 * radius) / num_voxels;
    lrf_radius = sqrt(3)*radius; // Such that the circumscribed sphere is obtained
    smoothing_factor = smoothing_kernel_width * (radius / num_voxels); // Equals half a voxel size so that 3X is 1.5 voxel
}

void SmoothNet::calculateLRF()
{    

    Image<float3, Host> v=_fusion->getAllVertex();

    std::cout<<"Reading key pts..."<<std::endl;
    readKeyPts(v);


    uint2 p;
    //create pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(p.x=0;p.x<v.size.x;p.x++)
    {
        for(p.y=0;p.y<v.size.y;p.y++)
        {
            float3 vertex=v[p];
            pcl::PointXYZ pt(vertex.x,vertex.y,vertex.z);
            cloud->push_back(pt);

        }
    }

    // Initialize the voxel grid
    flann::Matrix<float> voxel_coordinates = initializeGridMatrix(num_voxels, voxel_step_size, voxel_step_size, voxel_step_size);


    // Initialize the variables for the NN search and LRF computation
    std::vector<int> indices(cloud->width);
    std::vector<LRF> cloud_lrf(cloud->width);
    std::iota(indices.begin(), indices.end(), 0);
    std::vector <std::vector <int>> nearest_neighbors(cloud->width);
    std::vector <std::vector <int>> nearest_neighbors_smoothing(cloud->width);
    std::vector <std::vector <float>> nearest_neighbors_smoothing_dist(cloud->width);

    // Compute the local reference frame for the interes points (code adopted from https://www.researchgate.net/publication/310815969_TOLDI_An_effective_and_robust_approach_for_3D_local_shape_description
    // and not optimized)


    toldiComputeLRF(cloud,
                    evaluation_points,
                    lrf_radius, 3 * smoothing_factor,
                    cloud_lrf,
                    nearest_neighbors,
                    nearest_neighbors_smoothing,
                    nearest_neighbors_smoothing_dist);

    // Compute the SDV representation for all the points



    computeLocalDepthFeature(cloud,
                             evaluation_points,
                             nearest_neighbors,
                             cloud_lrf,
                             radius,
                             voxel_coordinates,
                             num_voxels,
                             smoothing_factor,
                             OUT_FILE_NAME);
}

bool SmoothNet::callCnn()
{
    char cmd[512];

//    sprintf(cmd,"python main_cnn.py --run_mode=test --evaluate_input_folder=%s --evaluate_output_folder=%s",
//            SDV_DIR,
//            CSV_DIR);
    sprintf(cmd,"./run_3dsmoothnet.sh %s %s",
            SDV_DIR,
            CSV_DIR);
    int status=system(cmd);

    if(status==0)
    {
        std::cout<<"Success"<<std::endl;
    }
    return status==0;
}

void SmoothNet::readKeyPts(const Image<float3, Host> &vert)
{
    int i=0;
    std::ifstream inFile(KEYPTS_F,std::ios::in);
    while(inFile.good() )
    {
        int idx;
        inFile>>idx;

        if(inFile.good())
        {
            uint2 pix;
            pix.x=idx/vert.size.y;
            pix.y=idx%vert.size.y;

            keypts[i]=pix;
            i++;
            evaluation_points.push_back(idx);
        }
    }
    inFile.close();
}

bool SmoothNet::readDescriptorCsv()
{
    std::ifstream inFile(CSV_FILE,std::ios::in);
    while(true)
    {
        std::string line;
        getline(inFile,line);
        if(inFile.eof())
        {
            break;
        }
        else if(!inFile.good())
        {
            inFile.close();
            return false;
        }        

        descr_t descr;
        std::istringstream lineStream(line);
        int i;
        //for descriptor of size 32
        for(i=0;i<32 && lineStream.good() ;i++)
        {
            std::string valueStr;
            getline(lineStream,valueStr,',');

            float value=std::stof(valueStr);
            descr.data[i]=value;            
        }
        if(i!=32 || !lineStream.eof() )
        {
            inFile.close();        
            return false;
        }
        descriptors.push_back(descr);
    }

    inFile.close();    
    std::cout<<"Red:"<<descriptors.size()<<" reatures."<<std::endl;
    return true;
}

void SmoothNet::calculateLRFPtr(uint2 pt)
{


}

bool SmoothNet::calculateLRFHost(uint2 pt,sMatrix3 &covar)
{

}












