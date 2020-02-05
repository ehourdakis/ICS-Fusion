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
#include <random>
#include<strings.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define KEYPTS_SIZE 200
#define SOCKET_PATH "/tmp/3dsmoothnet"

SmoothNet::SmoothNet(IcsFusion *f,kparams_t params)
    :_params(params),
      firstTime(true)
{
    _fusion=f;

    smoothing_kernel_width=1.75;
    num_voxels=16;
    radius=0.150;

    grid_size = num_voxels * num_voxels * num_voxels;
    voxel_step_size = (2 * radius) / num_voxels;
    lrf_radius = sqrt(3)*radius; // Such that the circumscribed sphere is obtained
    smoothing_factor = smoothing_kernel_width * (radius / num_voxels); // Equals half a voxel size so that 3X is 1.5 voxel

    counter_voxel = num_voxels * num_voxels * num_voxels;

    lrf=new float*[KEYPTS_SIZE];

    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        lrf[i] = new float[counter_voxel];
    }
}

bool SmoothNet::socketConnect()
{
    struct sockaddr_un addr;
    if ( (sock = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("socket error");
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path)-1);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        perror("connect error");
        exit(1);
        return false;
    }
    return true;
}

bool SmoothNet::sendLrfToSoc()
{
    int tmp[2];
    tmp[0]=KEYPTS_SIZE;
    tmp[1]=counter_voxel;
    write(sock,tmp,sizeof(int)*2);

    for (int i = 0; i < KEYPTS_SIZE; i++)
    {
        float *d=lrf[i];
        write(sock,d,sizeof(float)*counter_voxel);
    }

    char status=-1;
    while(true)
    {
        ssize_t s=recv(sock,&status,sizeof(char),0);
        if(s>0)
        {
            std::cout<<"status:"<<(int)status<<std::endl;
            break;
        }
        else if(s<0)
        {
            char buff[64];
            sprintf(buff,"Error reading from socket:%d",s);
            perror(buff);
            exit(1);
        }
    }
    return true;
}

SmoothNet::~SmoothNet()
{
    clear();
    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        delete lrf[i];
    }
    delete lrf;
}

void SmoothNet::clear()
{
    evaluation_points.clear();
    vertices.release();
    trackData.release();
}

void SmoothNet::loadFrameData(int frame)
{
    vertices=_fusion->getAllVertex();
    trackData=_fusion->getTrackData();

    if(!firstTime)
    {
        strcpy(prev_descr_file,descr_file);
        strcpy(prev_key_vert_file,key_vert_file);
    }


}

void SmoothNet::findKeyPts(int frame)
{
    evaluation_points.clear();

    std::vector<int> tmp_points;

    uint idx=0;
    uint2 pix;
    for(pix.x=0;pix.x<trackData.size.x;pix.x++)
    {
        for(pix.y=0;pix.y<trackData.size.y;pix.y++)
        {
            if(trackData[pix].result==-5)
            {
                tmp_points.push_back(idx);
            }
            idx++;
        }
    }

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int> distr(0, tmp_points.size());

    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        int oldIdx=distr(gen);
        while(tmp_points[oldIdx]==-1)
        {
            oldIdx=distr(gen);
        }
        evaluation_points.push_back(tmp_points[oldIdx]);
        tmp_points[oldIdx]=-1;
    }

    std::cout<<"Key pts size:"<<evaluation_points.size()<<std::endl;
}

void SmoothNet::calculateLRF(int frame)
{    
    std::cout<<"Reading key pts..."<<std::endl;

    uint2 p;
    //create pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(p.x=0;p.x<vertices.size.x;p.x++)
    {
        for(p.y=0;p.y<vertices.size.y;p.y++)
        {
            float3 vertex=vertices[p];
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

    // Compute the local reference frame for the interes points

    toldiComputeLRF(cloud,
                    evaluation_points,
                    lrf_radius, 3 * smoothing_factor,
                    cloud_lrf,
                    nearest_neighbors,
                    nearest_neighbors_smoothing,
                    nearest_neighbors_smoothing_dist);

    // Compute the SDV representation for all the points

    sprintf(sdv_file,"./data/sdv/frame%d.sdv",frame);

    computeLocalDepthFeature(cloud,
                             evaluation_points,
                             nearest_neighbors,
                             cloud_lrf,
                             radius,
                             voxel_coordinates,
                             counter_voxel,
                             smoothing_factor,
                             sdv_file,
                             lrf);
}

bool SmoothNet::callCnn(int frame)
{
    sprintf(descr_file,"./data/descr/frame%d.csv",frame);

    char cmd[512];

    sprintf(cmd,"./run_3dsmoothnet.sh %s %s",
            sdv_file,
            descr_file);

    std::cout<<cmd<<std::endl;
    int status=system(cmd);

    if(status==0)
    {
        std::cout<<"Success"<<std::endl;
    }
    return status==0;
}

void SmoothNet::readKeyPts()
{
    int i=0;
    std::ifstream inFile(KEYPTS_F,std::ios::in);
    while(inFile.good() )
    {
        int idx;
        inFile>>idx;

        if(inFile.good())
        {
            /*
            uint2 pix;
            pix.x=idx/vert.size.y;
            pix.y=idx%vert.size.y;            
            keypts[i]=pix;
            */
            i++;
            evaluation_points.push_back(idx);
        }
    }
    inFile.close();
}

bool SmoothNet::readDescriptorCsv(int frame)
{
    sprintf(descr_file,"./data/descr/frame%d.csv",frame);
    descriptors.clear();
    std::ifstream inFile(descr_file,std::ios::in);
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
    std::cout<<"Red:"<<descriptors.size()<<" features."<<std::endl;
    return true;
}

void SmoothNet::saveKeyPts(int frame)
{
    char out_file_name[256];
    sprintf(out_file_name,"./data/keypts/f_%d_keypoints.txt",frame);
    std::ofstream outFile(out_file_name,std::ios::out);

    for(int i=0;i<evaluation_points.size();i++)
    {
        outFile<<evaluation_points[i]<<"\n";
    }
    outFile.close();
}

void SmoothNet::saveKeyVertex(int frame)
{
    sprintf(key_vert_file,"./data/key_vertex/frame%d.csv",frame);
    std::ofstream outFile(key_vert_file,std::ios::out);

    float3 *vert_arr=vertices.data();
    for(int i=0;i<evaluation_points.size();i++)
    {
        int idx=evaluation_points[i];
        float3 vert=vert_arr[idx];

        outFile<<vert.x<<","<<vert.y<<","<<vert.z<<"\n";
    }
    outFile.close();
}

float SmoothNet::findTransformation(sMatrix4 &mat,float &rmse,int frame)
{
    rmse=0.0;
    if(firstTime)
    {
        firstTime=false;
        prevFrame=frame;
        return -2.0;
    }

    /*
    sprintf(trans_file,"./data/transformations/frame%d.txt",frame);

    char cmd[512];
    sprintf(cmd,"./run_ransac_demo.sh %d %d",
            prevFrame,
            frame);

    std::cout<<cmd<<std::endl;

    int status=system(cmd);

    prevFrame=frame;
    if(status!=0)
    {
        std::cout<<"Error"<<std::endl;
        return -1.0;
    }
    */
    std::ifstream inFile(trans_file,std::ios::in);
    float ret;

    inFile>>ret;
    inFile>>rmse;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            float f;
            inFile>>f;
            mat(i,j)=f;

        }

    }

    return ret;
}










