#include"smoothnet.h"

#include<fstream>
#include<eigen3/Eigen/Dense>
//#include <pcl/common/eigen.h>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

//#include <pcl/common/centroid.h>
#include<smoothnetcore.h>
#include <random>
#include<strings.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include"kernelscalls.h"

#define KEYPTS_SIZE 100
#define SOCKET_PATH "/tmp/3dsmoothnet"

SmoothNet::SmoothNet(IcsFusion *f,kparams_t params)
    :_params(params),
      firstTime(true),
      corresp(0)
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
    vertBuff1=new float3[KEYPTS_SIZE];
    vertBuff2=new float3[KEYPTS_SIZE];

    keyVert=vertBuff1;
    prevKeyVert=vertBuff2;

    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        lrf[i] = new float[counter_voxel];
    }
}

SmoothNet::~SmoothNet()
{
    clear();
    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        delete lrf[i];
    }
    delete lrf;
    delete vertBuff1;
    delete vertBuff2;
}

void SmoothNet::clear()
{
    evaluation_points.clear();
    vertices.release();
    trackData.release();
    if(corresp!=0)
    {
        delete corresp;
        corresp=0;
    }
}

void SmoothNet::loadFrameData(int frame)
{
    vertices=_fusion->getAllVertex();
    trackData=_fusion->getTrackData();

    float3 *tmp;
    tmp=prevKeyVert;
    prevKeyVert=keyVert;
    keyVert=tmp;
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
        int totalSend=0;
        int totalSize=sizeof(float)*counter_voxel;
        char *d=(char*)lrf[i];

        while(totalSend<totalSize)
        {
            int s=write(sock,d,totalSize-totalSend);
            d+=s;
            totalSend+=s;
        }
    }
    return true;
}

bool SmoothNet::findKeyPts(int frame)
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
    if(idx<KEYPTS_SIZE)
    {
        return false;
    }

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int> distr(0, tmp_points.size());

    for(int i=0;i<KEYPTS_SIZE;i++)
    {
        int idx=distr(gen);
        while(tmp_points[idx]==-1)
        {
            idx=distr(gen);
        }
        evaluation_points.push_back(tmp_points[idx]);

        uint2 pix;
        pix.x=tmp_points[idx]/vertices.size.y;
        pix.y=tmp_points[idx]%vertices.size.y;
        keyVert[i]=vertices[pix];

        tmp_points[idx]=-1;
    }
    return true;
}

void SmoothNet::calculateLRF(int frame)
{    
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
    computeLocalDepthFeature(cloud,
                             evaluation_points,
                             nearest_neighbors,
                             cloud_lrf,
                             radius,
                             voxel_coordinates,
                             counter_voxel,
                             smoothing_factor,                             
                             lrf);
}

bool SmoothNet::findDescriptors(int frame)
{
    bool b=findKeyPts(frame);
    if(!b)
        return false;

    calculateLRF(frame);
    sendLrfToSoc();
    sendKeyVertex(frame);
    if( receiveTf(last_tf,last_fitness,last_rmse) )
        receiveCorresp();

    return b;
}

sMatrix6 SmoothNet::calculateCov()
{
    return calculatePoint2PointCov(keyVert,
                                KEYPTS_SIZE,
                                prevKeyVert,
                                KEYPTS_SIZE,
                                corresp,
                                correspSize,
                                last_tf);

}

/*
bool SmoothNet::findTf(sMatrix4 &tf,
                       float &fitness,
                       float &rmse,
                       sMatrix6 &cov,
                       int _frame)
{
    findKeyPts(_frame);
    calculateLRF(_frame);
    sendLrfToSoc();
    sendKeyVertex(_frame);
    bool b=receiveTf(tf,fitness,rmse);
    std::cout<<fitness<<std::endl;
    if(!b)
    {
        return false;
    }
    if(!receiveCorresp() )
    {
            return false;
    }
    cov=calculatePoint2PointCov(keyVert,
                                KEYPTS_SIZE,
                                prevKeyVert,
                                KEYPTS_SIZE,
                                corresp,
                                correspSize,
                                tf);

    return true;
}
*/

bool SmoothNet::receiveCorresp()
{
    std::cout<<"rec corresp"<<std::endl;
    int size;
    char *buff=(char*)&size;
    //first receive the size
    int s=recv(sock,buff,sizeof(int),0 );
    if(s!=sizeof(int))
        return false;

    corresp = new int2[size];

    std::cout<<size<<std::endl;
    int totalSize=2*sizeof(int)*size;
    int totalRec=0;
    buff=(char*)corresp;

    while(totalRec<totalSize)
    {
        int s=recv(sock,buff,totalSize-totalRec,0 );
        if(s<0)
            return false;

        buff+=s;
        totalRec+=s;
    }
    correspSize=size;
    return true;

    /*
    for(int i=0;i<size;i++)
        std::cout<<corresp[i].x<<","<<corresp[i].y<<std::endl;
    */
}

bool SmoothNet::receiveTf(sMatrix4 &mat, float &fitness, float &rmse)
{
    fitness=-1;
    float fbuff[18];
    int totalSize=18*sizeof(float);
    int totalRec=0;
    char *buff=(char*)fbuff;
    while(totalRec<totalSize)
    {
        int s=recv(sock,buff,totalSize-totalRec,0 );
        if(s<0)
            return false;

        buff+=s;
        totalRec+=s;
    }
    fitness=fbuff[0];
    rmse=fbuff[1];
    for(int i=0;i<16;i++)
    {
        int x=i/4;
        int y=i%4;
        mat(x,y)=fbuff[i+2];
    }
    return fitness>0;
}

void SmoothNet::sendKeyVertex(int frame)
{
    char *buff=(char*)keyVert;

    int totalSend=0;
    int totalSize=evaluation_points.size()*3*sizeof(float);

    while(totalSend<totalSize)
    {
        int s=write(sock,buff,min(1024,totalSize-totalSend) );
        buff+=s;
        totalSend+=s;
    }
}

void SmoothNet::saveKeyPts(char *outFileName, int frame)
{
    //sprintf(out_file_name,"./data/keypts/f_%d_keypoints.txt",frame);
    std::ofstream outFile(outFileName,std::ios::out);

    for(int i=0;i<evaluation_points.size();i++)
    {
        outFile<<evaluation_points[i]<<"\n";
    }
    outFile.close();
}

void SmoothNet::saveKeyVertex(char *outFileName,int frame)
{
    //sprintf(key_vert_file,"./data/key_vertex/frame%d.csv",frame);
    std::ofstream outFile(outFileName,std::ios::out);
    for(int i=0;i<evaluation_points.size();i++)
    {
        int idx=evaluation_points[i];

        uint2 pix;
        pix.x=idx/vertices.size.y;
        pix.y=idx%vertices.size.y;
        float3 v=vertices[pix];
        outFile<<v.x<<","<<v.y<<","<<v.z<<"\n";
    }
    outFile.close();
}
