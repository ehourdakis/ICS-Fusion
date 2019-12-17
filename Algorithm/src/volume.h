#ifndef VOLUME_H
#define VOLUME_H

#include"cutil_math.h"
#include"utils.h"
#include"kparams.h"

#define IDX(a,b,c) a + b * _size.x + c * _size.x * _size.y
class Volume
{
    private:
        typedef float (Volume::*Fptr)(const uint3&) const;

    public:
        Volume()
        {
            _size = make_uint3(0);
            dim = make_float3(1);
            data = nullptr;
            color = nullptr;
        }

        __host__ __device__ uint3 size() const
        {
            return _size;
        }

        __host__ __device__ short2*  getDataPtr() const
        {
            return data;
        }

        __host__ __device__ float3 getOrigin() const
        {
            return dim;
        }

        __device__
        float2 operator[](const uint3 & pos) const
        {
            const short2 d = data[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y];
            return make_float2(d.x * 0.00003051944088f, d.y); //  / 32766.0f
        }

        __device__
        float3 getColor(const uint3 & pos) const
        {
            return color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y];
        }

        __device__
        float v(const uint3 & pos) const
        {
            return operator[](pos).x;
        }

        __device__
        float vs(const uint3 & pos) const
        {
            return data[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y].x;
        }

        __device__
        float red(const uint3 & pos) const
        {
            return color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y].x;
        }

        __device__
        float green(const uint3 & pos) const
        {
            return color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y].y;
        }

        __device__
        float blue(const uint3 & pos) const
        {
            return color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y].z;
        }

        __device__
        void set(const uint3 & pos, const float2 & d)
        {
            uint idx=pos.x + pos.y * _size.x + pos.z * _size.x * _size.y;
            data[idx] = make_short2(d.x * 32766.0f, d.y);
            color[idx] = make_float3(0.0,0.0,0.0);
        }

        __device__
        void set(const uint3 & pos, const float2 &d,const float3 &c)
        {
            data[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y] = make_short2(d.x * 32766.0f, d.y);
            color[pos.x + pos.y * _size.x + pos.z * _size.x * _size.y] = c;
        }

        __device__
        float3 pos(const uint3 & p) const
        {
            return make_float3((p.x + 0.5f) * dim.x / _size.x,
                               (p.y + 0.5f) * dim.y / _size.y, (p.z + 0.5f) * dim.z / _size.z);
        }

        __device__
        float interp(const float3 & pos) const
        {
            const Fptr fp = &Volume::vs;
            return generic_interp(pos,fp) * 0.00003051944088f;;
        }

        __device__
        float3 rgb_interp(const float3 & pos) const
        {

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
        float generic_interp(const float3 & pos,const Fptr fp) const
        {
            const float3 scaled_pos = make_float3((pos.x * _size.x / dim.x) - 0.5f,
                                                  (pos.y * _size.y / dim.y) - 0.5f,
                                                  (pos.z * _size.z / dim.z) - 0.5f);
            const int3 base = make_int3(floorf(scaled_pos));
            const float3 factor = fracf(scaled_pos);
            const int3 lower = max(base, make_int3(0));
            const int3 upper = min(base + make_int3(1),make_int3(_size) - make_int3(1));

            float tmp0 =( (this->*fp) (make_uint3(lower.x, lower.y, lower.z)) * (1 - factor.x) +
                        (this->*fp) (make_uint3(upper.x, lower.y, lower.z)) * factor.x ) * (1 - factor.y);
            float tmp1 =( (this->*fp) (make_uint3(lower.x, upper.y, lower.z)) * (1 - factor.x) +
                        (this->*fp) (make_uint3(upper.x, upper.y, lower.z)) * factor.x) * factor.y ;
            float tmp2 =( (this->*fp) (make_uint3(lower.x, lower.y, upper.z)) * (1 - factor.x) +
                        (this->*fp) (make_uint3(upper.x, lower.y, upper.z)) * factor.x) * (1 - factor.y);
            float tmp3 =( (this->*fp) (make_uint3(lower.x, upper.y, upper.z)) * (1 - factor.x) +
                        (this->*fp) (make_uint3(upper.x, upper.y, upper.z)) * factor.x) * factor.y;

    //        return ( (tmp0+tmp1) * (1 - factor.z) + (tmp2+tmp3) * factor.z ) * 0.00003051944088f;
            return ( (tmp0+tmp1) * (1 - factor.z) + (tmp2+tmp3) * factor.z ) ;
        }

        __device__ float3 grad(const float3 & pos) const;

        void init(uint3 s, float3 d)
        {
            _size = s;
            dim = d;
            cudaMalloc((void**)&data,_size.x * _size.y * _size.z * sizeof(short2));
            cudaMalloc(&color,_size.x * _size.y * _size.z * sizeof(float3));
            cudaMemset(data, 0, _size.x * _size.y * _size.z * sizeof(short2));
            cudaMemset(color, 0, _size.x * _size.y * _size.z * sizeof(float3));

        }

        void release()
        {
            if(data!=nullptr)
                cudaFree(data);
            if(data!=nullptr)
                cudaFree(color);

            data=nullptr;
            color=nullptr;
        }

    private:
//        typedef float (Volume::*Fptr)(const uint3&) const;

        uint3 _size;
        float3 dim;
        short2 *data;
        float3 *color;
};

//Usefull functions
// void dumpVolume(const char *  filename,const Volume volume);
void generateTriangles(std::vector<float3>& triangles,  const Volume volume, short2 *hostData);

void saveVoxelsToFile(const Volume volume,const kparams_t &params, std::string prefix);

#include"volume_impl.h"

#endif // VOLUME_H
