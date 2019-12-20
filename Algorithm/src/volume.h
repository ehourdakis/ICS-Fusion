#ifndef VOLUME_H
#define VOLUME_H

#include"cutil_math.h"
#include"utils.h"
#include"kparams.h"

//#define IDX(a,b,c) a + b * _size.x + c * _size.x * _size.y
class Volume
{
    private:
        typedef float (Volume::*Fptr)(const int3&) const;

    public:
        Volume()
        {
            _resolution = make_uint3(0);
            dim = make_float3(1);
            data = nullptr;
            color = nullptr;
        }

        __host__ __device__ uint3 getResolution() const
        {
            return _resolution;
        }

        __host__ __device__ float3 getVoxelSize() const
        {
            return voxelSize;
        }


        __host__ __device__ int3 getOffset() const
        {
            return _offset;
        }

        __host__ __device__ float3 getOffsetPos() const
        {
            return make_float3(_offset.x*voxelSize.x,
                               _offset.y*voxelSize.y,
                               _offset.z*voxelSize.z);
        }

        __host__ __device__ float3 getDimWithOffset() const
        {
            float3 ret=make_float3( dim.x+_offset.x*voxelSize.x,
                                    dim.y+_offset.y*voxelSize.y,
                                    dim.z+_offset.z*voxelSize.z);

            int3 v=maxVoxel();
            ret.x=(v.x)*voxelSize.x;
            ret.y=(v.y)*voxelSize.y;
            ret.z=(v.z)*voxelSize.z;
            return ret;
        }

        __host__ __device__ void addOffset(int3 off)
        {
            _offset.x+=off.x;
            _offset.y+=off.y;
            _offset.z+=off.z;
        }

        __host__ __device__ short2*  getDataPtr() const
        {
            return data;
        }

        __host__ __device__ float3 getDimensions() const
        {
            return dim;
        }

        __device__ size_t getPos(const int3 &p) const
        {
            int3 pos;

            if(p.x<minVoxel().x)
            {
                printf("Min x error:%d, %d\n",p.x,minVoxel().x);
            }
            if(p.x>=maxVoxel().x)
            {
                printf("Max x error:%d, %d\n",p.x,maxVoxel().x);
            }

            if(p.y<minVoxel().y)
            {
                printf("Min y error:%d, %d\n",p.y,minVoxel().y);
            }
            if(p.y>=maxVoxel().y)
            {
                printf("Max y error:%d, %d\n",p.y,maxVoxel().y);
            }

            if(p.z<minVoxel().z)
            {
                printf("Min z error:%d, %d\n",p.z,minVoxel().z);
            }
            if(p.z>=maxVoxel().z)
            {
                printf("Max z error:%d, %d\n",p.z,maxVoxel().z);
            }



//            pos.x=(p.x-_offset.x-1)%_resolution.x;
//            pos.y=(p.y-_offset.y-1)%_resolution.y;
//            pos.z=(p.z-_offset.z-1)%_resolution.z;
            if(p.x<0)
                pos.x=_resolution.x+p.x%(_resolution.x-1);
            else
                pos.x=p.x%(_resolution.x-1);

            if(p.y<0)
                pos.y=_resolution.x+p.y%(_resolution.y-1);
            else
                pos.y=p.y%(_resolution.y-1);

            if(p.z<0)
                pos.z=_resolution.x+p.z%(_resolution.z-1);
            else
                pos.z=p.z%(_resolution.z-1);

            return pos.x + pos.y * _resolution.x + pos.z * _resolution.x * _resolution.y;
        }

        __device__
        float2 operator[](const int3 & pos) const
        {
            const short2 d = data[getPos(pos)];
            return make_float2(d.x * 0.00003051944088f, d.y); //  / 32766.0f
        }

        __device__
        float3 getColor(const int3 & pos) const
        {
            return color[getPos(pos)];
        }

        __device__
        float vs(const int3 & pos) const
        {
            return data[getPos(pos)].x;
        }

        __device__
        float red(const int3 & pos) const
        {
            return color[getPos(pos)].x;
        }

        __device__
        float green(const int3 & pos) const
        {
            return color[getPos(pos)].y;
        }

        __device__
        float blue(const int3 & pos) const
        {
            //return color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y].z;
            return color[getPos(pos)].z;
        }

        __device__
        void set(const int3 & pos, const float2 & d)
        {
            size_t idx=getPos(pos);
            data[idx] = make_short2(d.x * 32766.0f, d.y);
            color[idx] = make_float3(0.0,0.0,0.0);
        }

        __device__
        void set(const int3 & pos, const float2 &d,const float3 &c)
        {
            size_t p=getPos(pos);
            data[p] = make_short2(d.x * 32766.0f, d.y);
            color[p] = c;
        }

        __device__
        float3 pos(const int3 & p) const
        {
//            return make_float3( ( (p.x + 0.5f) * dim.x / _resolution.x)+_offset.x ,
//                                ( (p.y + 0.5f) * dim.y / _resolution.y)+_offset.y ,
//                                ( (p.z + 0.5f) * dim.z / _resolution.z)+_offset.z);
            return make_float3( ( (p.x + 0.5f) * voxelSize.x),
                                ( (p.y + 0.5f) * voxelSize.y),
                                ( (p.z + 0.5f) * voxelSize.z));
        }

        __device__
        float interp(const float3 & pos) const
        {
            const Fptr fp = &Volume::vs;
            return generic_interp(pos,fp) * 0.00003051944088f;
        }

        __device__
        float3 rgb_interp(const float3 &p) const
        {
            float3 pos=p;/*
            pos.x=pos.x+_offset.x*voxelSize.x;
            pos.y=pos.x+_offset.y*voxelSize.y;
            pos.z=pos.x+_offset.z*voxelSize.z;*/

            float3 rgb;
            const Fptr red_ptr = &Volume::red;
            rgb.x=generic_interp(pos,red_ptr);

            const Fptr green_ptr = &Volume::green;
            rgb.y=generic_interp(pos,green_ptr);

            const Fptr blue_ptr = &Volume::blue;
            rgb.z=generic_interp(pos,blue_ptr);
            return rgb;
        }

        __device__
        float generic_interp(const float3 & pos,const Fptr fp) const;

        __device__ float3 grad(const float3 & pos) const;

        void init(uint3 s, float3 d, int3 sliceSize)
        {
            _resolution = s;
            dim = d;
            _sliceSize=sliceSize;
            cudaMalloc((void**)&data,_resolution.x * _resolution.y * _resolution.z * sizeof(short2));
            cudaMalloc(&color,_resolution.x * _resolution.y * _resolution.z * sizeof(float3));
            cudaMemset(data, 0, _resolution.x * _resolution.y * _resolution.z * sizeof(short2));
            cudaMemset(color, 0, _resolution.x * _resolution.y * _resolution.z * sizeof(float3));

            voxelSize=dim/_resolution;
            /*
            voxelSize.x=dim.x/_resolution.x;
            voxelSize.y=dim.y/_resolution.y;
            voxelSize.z=dim.z/_resolution.z;
            */


            float3 offsetF=make_float3( (-4.f/voxelSize.x)+0.5f,
                                        (-4.f/voxelSize.y)+0.5f,
                                        (-4.f/voxelSize.z)+0.5f);
            _offset=make_int3(int(offsetF.x),int(offsetF.y),int(offsetF.z));

            printf("%f %f %f\n",offsetF.x,offsetF.y,offsetF.z);
            printf("%d %d %d\n",_offset.x,_offset.y,_offset.z);

            printf("%d %d %d\n",_resolution.x+_offset.x,_resolution.y+_offset.y,_resolution.z+_offset.z);
            _offset=make_int3(0,0,0);
        }

        __host__ __device__ int3 minVoxel() const
        {
            return _offset;
        }

        __host__ __device__ int3 maxVoxel() const
        {
            return make_int3( int(_resolution.x)+_offset.x,
                              int(_resolution.y)+_offset.y,
                              int(_resolution.z)+_offset.z);
        }

        void release()
        {
            if(data!=nullptr)
                cudaFree(data);
            if(color!=nullptr)
                cudaFree(color);

            data=nullptr;
            color=nullptr;
        }

    private:
//        typedef float (Volume::*Fptr)(const uint3&) const;

        uint3 _resolution;
        float3 dim;
        int3 _sliceSize;
        float3 voxelSize;
        int3 _offset;

        short2 *data;
        float3 *color;


};

//Usefull functions
// void dumpVolume(const char *  filename,const Volume volume);
void generateTriangles(std::vector<float3>& triangles,  const Volume volume, short2 *hostData);

void saveVoxelsToFile(const Volume volume,const kparams_t &params, std::string prefix);

#include"volume_impl.h"

#endif // VOLUME_H
