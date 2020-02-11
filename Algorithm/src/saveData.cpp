#include"saveData.h"
#include<iostream>
#include<fstream>
#include <Eigen/Dense>


void saveVoxelsToFile(char *fileName,const Volume volume,const kparams_t &params)
{
    //TODO this function needs cleanup and speedup
    std::cout<<"Saving TSDF voxel grid values to disk("<<fileName<<")"<< std::endl;

    std::ofstream outFile(fileName, std::ios::out);
    float dimensions[3];
    dimensions[0]=float(volume.getResolution().x);
    dimensions[1]=float(volume.getResolution().y);
    dimensions[2]=float(volume.getResolution().z);

    outFile<<dimensions[0]<<std::endl;
    outFile<<dimensions[1]<<std::endl;
    outFile<<dimensions[2]<<std::endl;

    float origin[3];
    origin[0]=0.0;
    origin[1]=0.0;
    origin[2]=0.0;

    outFile<<origin[0]<<std::endl;
    outFile<<origin[1]<<std::endl;
    outFile<<origin[2]<<std::endl;

    float vox_size=float(params.volume_size.x)/float(params.volume_resolution.x);
    outFile<<vox_size<<std::endl;
    outFile<<params.mu<<std::endl;

    short2 *voxel_grid_cpu=new short2[volume.getResolution().x*volume.getResolution().y*volume.getResolution().z];

    cudaMemcpy(voxel_grid_cpu, volume.getDataPtr(),
                   volume.getResolution().x*volume.getResolution().y*volume.getResolution().z* sizeof(short2),
                   cudaMemcpyDeviceToHost);

    //for(int i=0;i<params.volume_resolution.x*params.volume_resolution.y*params.volume_resolution.z;i++)

    uint3 pos;
    for(pos.x=0;pos.x<volume.getResolution().x;pos.x++)
    {
        for(pos.y=0;pos.y<volume.getResolution().y;pos.y++)
        {
            for(pos.z=0;pos.z<volume.getResolution().z;pos.z++)
            {
                uint arrayPos=volume.getPos(pos);
                short2 data=voxel_grid_cpu[arrayPos];
                float value=float(data.x)/32766.0f;
                outFile<<value<<'\n';
            }
        }
    }

    outFile.close();

    std::cout<<"Saving done."<<std::endl;
    delete [] voxel_grid_cpu;
}

void savePoses(char *fileName,const std::vector<sMatrix4> &poses)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    Eigen::Matrix3f rot;
    for(uint p=0;p<poses.size();p++)
    {
        sMatrix4 pose=poses[p];
        for (int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                rot(i,j)=pose(i,j);
            }
        }
        Eigen::Vector3f rotV = rot.eulerAngles(2, 1, 0);

        file<<pose(0,3)<<','<<pose(1,3)<<','<<pose(2,3)<<" " ;
        file<<rotV(0)<<','<<rotV(1)<<','<<rotV(2)<<'\n';
    }

    file.close();
}

void savePose(char *fileName,const sMatrix4 &pose)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    
    Eigen::Matrix3f rot;
    

    
    for (int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rot(i,j)=pose(i,j);
        }
    }
    Eigen::Vector3f rotV = rot.eulerAngles(2, 1, 0);
    file<<pose(0,3)<<','<<pose(1,3)<<','<<pose(2,3)<<" " ;
    file<<rotV(0)<<','<<rotV(1)<<','<<rotV(2)<<'\n';
    /*

    Eigen::Matrix3f rot;

    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            //rot(i,j)=pose(i,j);
            file<<pose(i,j)<<' ';
        }
        file<<'\n';
    }    
    */
    file.close();
}

void saveVertexPly(char *fileName,const Image<float3, Host> &vert)
{
    std::ofstream out_file(fileName,std::ofstream::binary);
    uint2 size=vert.size;

    char buf[128];
    sprintf(buf,"ply\n");
    out_file.write(buf,strlen(buf));

    sprintf(buf,"format binary_little_endian 1.0\n");
    out_file.write(buf,strlen(buf));

    sprintf(buf,"element vertex %d\n",size.x*size.y);
    out_file.write(buf,strlen(buf));


    sprintf(buf,"property float x\nproperty float y\nproperty float z\n");
    out_file.write(buf,strlen(buf));

    sprintf(buf,"end_header\n");
    out_file.write(buf,strlen(buf));

    float vert_val[3];
    for(int x=0;x<size.x;x++)
    {
        for(int y=0;y<size.y;y++)
        {
            uint2 pos=make_uint2(x,y);
            float3 val=vert[pos];
            vert_val[0]=val.x;
            vert_val[1]=val.y;
            vert_val[2]=val.z;
            out_file.write( (char*)vert_val,sizeof(float)*3 );
        }
    }
    out_file.close();
}
