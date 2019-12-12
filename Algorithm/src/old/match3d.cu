#include"match3d.h"
#include<iostream>
#include<fstream>
#include<KDTree.hpp>
#include<vector>
#include<utils.hpp>
#include"kernels.h"
#include<response.h>
//#include"utils.h"

#define sq(x) x*x

static dim3 imageBlock = dim3(32, 16);

// #pragma message MARVIN_DIR
// #pragma message FULL_NET_STRUCTURE
// #pragma message FULL_NET_WEIGHTS


#include"constant_parameters.h"
Match3D::Match3D(kparams_t p)
    :net(NET_STRUCTURE),
     num_keypts(500),
     batch_size(50),
     desc_size(512),
     params(p)
{
    std::cout<<NET_STRUCTURE<<std::endl;
    std::cout<<NET_WEIGHTS<<std::endl;

    net.MallocForTest();
    net.loadWeights(NET_WEIGHTS);

    rData = net.getResponse("data");
    rFeat = net.getResponse("feat");
}

Match3D::~Match3D()
{
}

//TODO cudify me
void Match3D::createTempKeyPoints(uint3 *kp,uint3 size,uint num)
{

    float minZ=1.5;
    float3 origin;
    float3 voxel_size;

    voxel_size.x=float(params.volume_resolution.x)/float(params.volume_size.x);
    voxel_size.y=float(params.volume_resolution.y)/float(params.volume_size.y);
    voxel_size.z=float(params.volume_resolution.z)/float(params.volume_size.z);


    origin.x=params.volume_direction.x*voxel_size.x;
    origin.y=params.volume_direction.y*voxel_size.y;
    origin.z=params.volume_direction.z*voxel_size.z;

    int minZvoxels=minZ*voxel_size.z;
    
    //TODO write it on cuda
    for(int i=0;i<num;i++)
    {
        /*
         * First generate random keypts with maximum z (only x,y are random)
         * Then convert then to spherical cordinates (f,th,r).
         * then we generate a new random radious(randR).
         * We use the previous radious as the maximum value.
         * Finally we convert back to cartesian
         */ 
        float x=GetRandomFloat(15, (float)size.x-15);
        float y=GetRandomFloat(15, (float)size.y-15);
        float z=(float)size.z-15;

        x-=origin.x;
        y-=origin.y;
        z-=origin.z;
        
        //find spherical cord.
        float r=sqrt(sq(x)+sq(y)+sq(z) );
        float f=atan(y/x); //TODO atan2?
        float th=acos(z/r);

        float randR=GetRandomFloat(minZvoxels+15, r);
        x=randR*sin(th)*cos(f);
        y=randR*sin(th)*sin(f);
        z=randR*cos(th);


        kp[i].x=(uint) (x+origin.x);
        kp[i].y=(uint) (y+origin.y);
        kp[i].z=(uint) (z+origin.z);
    }
}
//TODO cudify me
void Match3D::createKeyPoints(uint3 *kp,uint3 size,uint num)
{
    //TODO write it on cuda
    for(int i=0;i<num;i++)
    {
        float x=GetRandomFloat(15, (float)size.x-15);
        float y=GetRandomFloat(15, (float)size.y-15);
        float z=GetRandomFloat(15, (float)size.z-15);

        kp[i].x=(uint) (x);
        kp[i].y=(uint) (y);
        kp[i].z=(uint) (z);

    }
}

void Match3D::findDescriptors(Volume vol, int num, uint3 *kp,
                              float *desc, char *empty)
{
    uint3 *keyPointGpu;
    char *isEmpty;
    cudaMalloc(&keyPointGpu,batch_size*sizeof(uint3));
    cudaMalloc(&isEmpty,batch_size*sizeof(uchar));

    char *desc_buf=(char*)desc;
    char *is_empty_buf=empty;
    int i;
    for(i=0;i<num/batch_size;i++)
    {
        uint3 *batch_kp=kp+i*batch_size;
        cudaMemcpy(keyPointGpu,batch_kp,sizeof(uint3)*batch_size,cudaMemcpyHostToDevice);

        //TODO more clever kernel workload
        uint3 grid=make_uint3(30,30,30);
        copyVolumeData2<<<1,batch_size>>>(keyPointGpu,vol,rData->dataGPU,isEmpty );
        net.forward();
        printCUDAError();

        cudaMemcpy(desc_buf, rFeat->dataGPU, rFeat->numBytes(), cudaMemcpyDeviceToHost);
        desc_buf+=rFeat->numBytes();

        cudaMemcpy(is_empty_buf,isEmpty, batch_size*sizeof(char), cudaMemcpyDeviceToHost);
        is_empty_buf+=batch_size;
//        printf("B %d\n",rFeat->numBytes())
    }
std::cout<<"EDO"<<std::endl;
    int rem=num%batch_size;
    uint3 *batch_kp=kp+i*batch_size;
    uint3 grid=make_uint3(30,30,30);
    cudaMemcpy(keyPointGpu,batch_kp,sizeof(uint3)*rem,cudaMemcpyHostToDevice);
    copyVolumeData2<<<1,rem>>>(keyPointGpu,vol,rData->dataGPU,isEmpty );


    net.forward();
    std::cout<<"EDO2"<<std::endl;
    printCUDAError();
    cudaMemcpy(desc_buf, rFeat->dataGPU, rem*desc_size, cudaMemcpyDeviceToHost);
//    desc_buf+=rFeat->numBytes();

    cudaMemcpy(is_empty_buf,isEmpty, rem*sizeof(char), cudaMemcpyDeviceToHost);
    is_empty_buf+=batch_size;


    cudaFree(keyPointGpu);
    
//    pointVec points;
//    points.reserve(num);

//    for(int i=0;i<num;i++)
//    {
//        point_t descV;
//        descV.resize(desc_size);
//        memcpy(descV.data(),desc+desc_size*i,desc_size);
//        points.push_back(descV);
//    }
}

void Match3D::saveKeypoints(std::string fileName,uint3 *keypts,
                            char *isEmpty,int num_keypts)
{
    std::cout << "Saving keypoints to disk("<<fileName<<")..."<< std::endl;
    std::ofstream keypts_out_file(fileName, std::ios::out);

    for (int i = 0; i < num_keypts; i++)
    {
//         if(isEmpty[i])
//             continue;
        keypts_out_file<<keypts[i].x<<" "
                       <<keypts[i].y<<" "
                       <<keypts[i].z<<"\n";
    }
    keypts_out_file.close();
}

// Save 3DMatch descriptors as binary file (Nx512 float array, row-major order)
void Match3D::saveDescriptors(std::string fileName,float *desc,char *isEmpty,int num)
{

    int emptySize=0;
    for(int i=0;i<num;i++)
    {
        if(isEmpty[i])
            emptySize++;
    }

    std::cout << "Saving 3DMatch descriptors to disk (desc.3dmatch.bin)..." << std::endl;
    std::ofstream desc_out_file(fileName, std::ios::binary | std::ios::out);
    float desc_sizef = (float) desc_size;

    desc_out_file<<(float) (num-emptySize)<<'\n';
    desc_out_file<<desc_sizef<<'\n';

    float *d=desc;
    for(int i=0;i<num;i++)
    {
//         if(isEmpty[i])
//             continue;

        d=desc+i*desc_size;
        for(int j=0;j<desc_size;j++)
            desc_out_file<<d[j]<<'\n';
    }
    desc_out_file.close();
}
