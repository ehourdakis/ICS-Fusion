#ifndef UTILS_H
#define UTILS_H

#include<limits>

#if defined(__GNUC__)
// circumvent packaging problems in gcc 4.7.0
#undef _GLIBCXX_ATOMIC_BUILTINS 
#undef _GLIBCXX_USE_INT128 
// need c headers for __int128 and uint16_t
#include <limits.h>
#endif

#include <stdint.h>
#include <iostream>
#include <vector>

#include <vector_types.h>
#include "cutil_math.h"

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/GR_SVD.h>

#include <iomanip>
#include <cuda_runtime.h>
#include"image.h"

#define INVALID -2   // this is used to mark invalid entries in normal or vertex maps

#define TICK(str)    {static const std::string str_tick = str; \
    if (print_kernel_timing) {clock_gettime(CLOCK_MONOTONIC, &tick_clockData);}

#define TOCK()        if (print_kernel_timing) {cudaDeviceSynchronize(); \
    clock_gettime(CLOCK_MONOTONIC, &tock_clockData); \
    std::cerr<< str_tick << " ";\
    if((tock_clockData.tv_sec > tick_clockData.tv_sec) && (tock_clockData.tv_nsec >= tick_clockData.tv_nsec)) std::cerr<< tock_clockData.tv_sec - tick_clockData.tv_sec << std::setfill('0') << std::setw(9);\
    std::cerr  << (( tock_clockData.tv_nsec - tick_clockData.tv_nsec) + ((tock_clockData.tv_nsec<tick_clockData.tv_nsec)?1000000000:0)) << std::endl;}}

extern bool print_kernel_timing;

extern struct timespec tick_clockData;
extern struct timespec tock_clockData;

__forceinline__ __host__ __device__ int  signf(const float a)
{
    return a>0?1:-1;
}


__forceinline__ __host__ __device__ int  sign(const int a)
{
    return a>0?1:-1;
}

__forceinline__ __host__ __device__ void  swapf(float &f1,float &f2)
{
    float tmp=f2;
    f2=f1;
    f1=tmp;
}


__forceinline__ __host__ __device__ float sq(const float x)
{
    return x * x;
}

inline int printCUDAError()
{
    cudaError_t error = cudaGetLastError();
    if (error)
        std::cout <<"[CUDA ERR]"<<cudaGetErrorString(error) << std::endl;
    return error;
}

inline int divup(int a, int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

inline dim3 divup(uint2 a, dim3 b)
{
    return dim3(divup(a.x, b.x), divup(a.y, b.y));
}
inline dim3 divup(dim3 a, dim3 b)
{
    return dim3(divup(a.x, b.x), divup(a.y, b.y), divup(a.z, b.z));
}

struct Matrix4
{
    float4 data[4];
    //Identity matrix
    __host__  __device__ Matrix4()
    {
        this->data[0] = make_float4(1,0,0,0);
        this->data[1] = make_float4(0,1,0,0);
        this->data[2] = make_float4(0,0,1,0);
        this->data[3] = make_float4(0,0,0,1);
    }
    
    __host__  __device__ Matrix4(float x1,float y1,float z1,float w1,
            float x2,float y2,float z2,float w2,
            float x3,float y3,float z3,float w3,
            float x4,float y4,float z4,float w4
    )
    {
        data[0]=make_float4(x1,y1,z1,w1);
        data[1]=make_float4(x2,y2,z2,w2);
        data[2]=make_float4(x3,y3,z3,w3);
        data[3]=make_float4(x4,y4,z4,w4);
    }

    /*This is stupid. Just use two dimensions array*/
    float& __host__  __device__ operator () (int i,int j)
    {
        if(j==0)
            return this->data[i].x;
        else if(j==1)
            return this->data[i].y;
        else if(j==2)
            return this->data[i].z;
        else //if(j==3)
            return this->data[i].w;
    }


    /*This is also stupid.*/
    const float& __host__  __device__ operator () (int i,int j) const
    {
        if(j==0)
            return this->data[i].x;
        else if(j==1)
            return this->data[i].y;
        else if(j==2)
            return this->data[i].z;
        else //if(j==3)
            return this->data[i].w;
    }

    __host__ __device__ Matrix4(Matrix4 * src)
    {
        this->data[0] = make_float4(src->data[0].x, src->data[0].y,
                src->data[0].z, src->data[0].w);
        this->data[1] = make_float4(src->data[1].x, src->data[1].y,
                src->data[1].z, src->data[1].w);
        this->data[2] = make_float4(src->data[2].x, src->data[2].y,
                src->data[2].z, src->data[2].w);
        this->data[3] = make_float4(src->data[3].x, src->data[3].y,
                src->data[3].z, src->data[3].w);
    }

    inline __host__  __device__ float3 get_translation() const
    {
        return make_float3(data[0].w, data[1].w, data[2].w);
    }
};
typedef Matrix4 sMatrix4;

extern sMatrix4 T_B_P;
extern sMatrix4 invT_B_P;

struct sMatrix6
{
    float data[6*6];

    __host__ __device__ sMatrix6()
    {
        for(int i=0;i<6;i++)
        {
            for(int j=0;j<6;j++)
            {
                int idx=6*i+j;
                if(i==j)
                    data[idx]=1.0;
                else
                    data[idx]=0.0;
            }
        }
    }

    static __host__ __device__ sMatrix6 zeros()
    {
        sMatrix6 ret;
        for(int i=0;i<6*6;i++)
            ret.data[i]=0.0;
        return ret;
    }

    inline __host__  __device__ float& operator () (int i,int j)
    {
        int idx=6*i+j;
        return data[idx];
    }
    
    inline __host__  __device__ const float& operator () (int i,int j) const
    {
        int idx=6*i+j;
        return data[idx];
    }
};

class sMatrix3
{
    public:
    float data[3*3];
    __host__ __device__ sMatrix3()
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                int idx=3*i+j;
                if(i==j)
                    data[idx]=1.0;
                else
                    data[idx]=0.0;
            }
        }
    }

    inline __host__  __device__ float& operator () (int i,int j)
    {
        int idx=3*i+j;
        return data[idx];
    }

    inline __host__  __device__ const float& operator () (int i,int j) const
    {
        int idx=3*i+j;
        return data[idx];
    }

    static sMatrix3 zeros()
    {
        sMatrix3 mat;
        for(int i=0;i<3*3;i++)
            mat.data[i]=0.0;
        return mat;
    }
};

inline __host__  __device__ sMatrix6 operator+(const sMatrix6 &c1, const sMatrix6 &c2)
{ 
    sMatrix6 ret;
    for(int i=0;i<36;i++)
    {
        ret.data[i]=c1.data[i]+c2.data[i];
    }   
    return ret;
}

inline __host__  __device__ sMatrix4 operator-(const sMatrix4 &c1, const sMatrix4 &c2)
{ 
    sMatrix4 ret;
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j)=c1(i,j)-c2(i,j);
    return ret;
}

inline __host__  __device__ sMatrix3 operator-(const sMatrix3 &c1, const sMatrix3 &c2)
{
    sMatrix3 ret;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ret(i,j)=c1(i,j)-c2(i,j);
    return ret;
}



std::ostream & operator<<(std::ostream & out, const Matrix4 & m);
Matrix4 operator*(const Matrix4 & A, const Matrix4 & B);
Matrix4 inverse(const Matrix4 & A);

inline __host__  __device__ float4 operator*(const Matrix4 & M,const float4 & v)
{
    return make_float4(dot(M.data[0], v), dot(M.data[1], v), dot(M.data[2], v),
            dot(M.data[3], v));
}

inline __host__  __device__ float3 operator*(const Matrix4 & M,const float3 & v)
{
    return make_float3(dot(make_float3(M.data[0]), v) + M.data[0].w,
            dot(make_float3(M.data[1]), v) + M.data[1].w,
            dot(make_float3(M.data[2]), v) + M.data[2].w);
}

inline __host__  __device__ float3 rotate(const Matrix4 & M, const float3 & v)
{
    return make_float3(dot(make_float3(M.data[0]), v),
                       dot(make_float3(M.data[1]), v),
                       dot(make_float3(M.data[2]), v));
}

inline Matrix4 getCameraMatrix(const float4 & k) {
    Matrix4 K;
    K.data[0] = make_float4(k.x, 0, k.z, 0);
    K.data[1] = make_float4(0, k.y, k.w, 0);
    K.data[2] = make_float4(0, 0, 1, 0);
    K.data[3] = make_float4(0, 0, 0, 1);
    return K;
}

inline Matrix4 getInverseCameraMatrix(const float4 & k)
{
    Matrix4 invK;
    invK.data[0] = make_float4(1.0f / k.x, 0, -k.z / k.x, 0);
    invK.data[1] = make_float4(0, 1.0f / k.y, -k.w / k.y, 0);
    invK.data[2] = make_float4(0, 0, 1, 0);
    invK.data[3] = make_float4(0, 0, 0, 1);
    return invK;
}








template<typename OTHER>
inline void image_copy(Ref & to, const OTHER & from, uint size) {
    to.data = from.data;
}

inline void image_copy(Host & to, const Host & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyHostToHost);
}

inline void image_copy(Host & to, const Device & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyDeviceToHost);
}

inline void image_copy(Host & to, const HostDevice & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyHostToHost);
}

inline void image_copy(Device & to, const Ref & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyDeviceToDevice);
}

inline void image_copy(Device & to, const Host & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyHostToDevice);
}

inline void image_copy(Device & to, const Device & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyDeviceToDevice);
}

inline void image_copy(Device & to, const HostDevice & from, uint size) {
    cudaMemcpy(to.data, from.getDevice(), size, cudaMemcpyDeviceToDevice);
}

inline void image_copy(HostDevice & to, const Host & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyHostToHost);
}

inline void image_copy(HostDevice & to, const Device & from, uint size) {
    cudaMemcpy(to.getDevice(), from.data, size, cudaMemcpyDeviceToDevice);
}

inline void image_copy(HostDevice & to, const HostDevice & from, uint size) {
    cudaMemcpy(to.data, from.data, size, cudaMemcpyHostToHost);
}

struct TrackData
{
    int result;
    float error;
    float J[6];
};

bool __forceinline__ __host__ __device__ operator==(const TrackData &d1,const TrackData &d2)
{
    return d1.result==d2.result;
}


template<typename P>
inline Matrix4 toMatrix4(const TooN::SE3<P> & p)
{
    const TooN::Matrix<4, 4, float> I = TooN::Identity;
    Matrix4 R;
    TooN::wrapMatrix<4, 4>(&R.data[0].x) = p * I;
    return R;
}

template<typename P>
inline sMatrix4 tosMatrix4(const TooN::SE3<P> & p)
{
    const TooN::Matrix<4, 4, float> I = TooN::Identity;
    sMatrix4 R;
    TooN::wrapMatrix<4, 4>(&R.data[0].x) = p * I;
    return R;
}

template<typename P, typename A>
TooN::Matrix<6> makeJTJ(const TooN::Vector<21, P, A> & v)
{
    TooN::Matrix<6> C = TooN::Zeros;
    C[0] = v.template slice<0, 6>();
    C[1].template slice<1, 5>() = v.template slice<6, 5>();
    C[2].template slice<2, 4>() = v.template slice<11, 4>();
    C[3].template slice<3, 3>() = v.template slice<15, 3>();
    C[4].template slice<4, 2>() = v.template slice<18, 2>();
    C[5][5] = v[20];

    for (int r = 1; r < 6; ++r)
        for (int c = 0; c < r; ++c)
            C[r][c] = C[c][r];

    return C;
}

template<typename T, typename A>
TooN::Vector<6> solve(const TooN::Vector<27, T, A> & vals) {
    const TooN::Vector<6> b = vals.template slice<0, 6>();
    const TooN::Matrix<6> C = makeJTJ(vals.template slice<6, 21>());

    TooN::GR_SVD<6, 6> svd(C);
    return svd.backsub(b, 1e6);
}

class Timestamp
{
    public:
        uint32_t sec;
        uint32_t nsec;
        
        Timestamp(uint32_t s,uint32_t ns) :sec(s),nsec(ns){}
};

inline Matrix4 operator*(const sMatrix4 & A, const sMatrix4 & B)
{
    Matrix4 R;
    TooN::wrapMatrix<4, 4>(&R.data[0].x) = TooN::wrapMatrix<4, 4>(&A.data[0].x)
            * TooN::wrapMatrix<4, 4>(&B.data[0].x);
    return R;
}

inline sMatrix3 operator*(const sMatrix3 & A, const sMatrix3 & B)
{
    sMatrix3 R;
    TooN::wrapMatrix<3, 3>(&R.data[0]) = TooN::wrapMatrix<3, 3>(&A.data[0])
            * TooN::wrapMatrix<3, 3>(&B.data[0]);
    return R;
}

inline sMatrix6 operator*(const sMatrix6 & A, const sMatrix6 & B)
{
    sMatrix6 R;
    TooN::wrapMatrix<6, 6>(&R.data[0]) = TooN::wrapMatrix<6, 6>(&A.data[0])
            * TooN::wrapMatrix<6, 6>(&B.data[0]);
    return R;
}

inline sMatrix6 operator*(const sMatrix6 & A, const float f)
{
    sMatrix6 R;
    for(int i=0;i<36;i++)
        R.data[i]=A.data[i]*f;
    return R;
}

inline sMatrix3 operator*(const sMatrix3 & A, const float f)
{
    sMatrix3 R;
    for(int i=0;i<9;i++)
        R.data[i]=A.data[i]*f;
    return R;
}

inline sMatrix4 inverse(const sMatrix4 & A)
{
    static TooN::Matrix<4, 4, float> I = TooN::Identity;
    TooN::Matrix<4, 4, float> temp = TooN::wrapMatrix<4, 4>(&A.data[0].x);
    Matrix4 R;
    TooN::wrapMatrix<4, 4>(&R.data[0].x) = TooN::gaussian_elimination(temp, I);
    return R;
}

inline sMatrix6 inverse(const sMatrix6 & A)
{
    static TooN::Matrix<6, 6, float> I = TooN::Identity;
    TooN::Matrix<6, 6, float> temp = TooN::wrapMatrix<6, 6>(&A.data[0]);
    sMatrix6 R;
    TooN::wrapMatrix<6, 6>(&R.data[0]) = TooN::gaussian_elimination(temp, I);
    return R;
}

inline sMatrix3 wedge(float *v)
{
    sMatrix3 skew;
    for(int i=0;i<3*3;i++)
        skew.data[i]=0.0;

    //skew = Matrix3d::zero();
    skew(0, 1) = -v[2];
    skew(0, 2) = v[1];
    skew(1, 2) = -v[0];
    skew(1, 0) = v[2];
    skew(2, 0) = -v[1];
    skew(2, 1) = v[0];

    return skew;

}

inline sMatrix3 transpose( const sMatrix3 &mat)
{
    sMatrix3 ret;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ret(j,i)=mat(i,j);
    return ret;
}

inline std::ostream & operator<<(std::ostream & out, const Matrix4 & m)
{
    for (unsigned i = 0; i < 4; ++i)
        out << m.data[i].x << "  " << m.data[i].y << "  " << m.data[i].z << "  "
            << m.data[i].w << "\n";
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const float3 f)
{
    out<<"("<<f.x<<","<<f.y<<","<<f.z<<")";
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const int3 f)
{
    out<<"("<<f.x<<","<<f.y<<","<<f.z<<")";
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const sMatrix6 & m)
{
    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            out << m(i,j)<< "  ";
        }
        out<<"\n";
    }

    return out;
}

inline std::ostream & operator<<(std::ostream & out, const sMatrix3 & m)
{
    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            out << m(i,j)<< "  ";
        }
        out<<"\n";
    }

    return out;
}

inline sMatrix6 tr(const sMatrix6 &mat)
{
    sMatrix6 ret;
    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            ret(j,i)=mat(i,j);
        }
    }
    return ret;
}

inline void synchroniseDevices()
{
    cudaDeviceSynchronize();
}

inline sMatrix4 fromVisionCord(const sMatrix4 &mat)
{
    return invT_B_P*mat*T_B_P;
//    return ret;
}

inline sMatrix4 toVisionCord(const sMatrix4 &mat)
{
    return T_B_P*mat*invT_B_P;
//    return ret;
}

inline float3 __host__  __device__ fromVisionCordV(const float3 &v)
{
    float3 ret=make_float3( v.z,-v.x,-v.y);
    return ret;
}

inline float3 toVisionCordV(const float3 &v)
{
    sMatrix4 mat;
    mat(0,3)=v.x;
    mat(1,3)=v.y;
    mat(2,3)=v.z;

    mat=toVisionCord(mat);
    float3 ret=mat.get_translation();
//    float3 ret=make_float3( -v.y,-v.z,v.x);
    return ret;
}

//inline sMatrix4 toVisionCord(const sMatrix4 &mat)
//{
////    static sMatrix4 invT_B_P=inverse(T_B_P);
//    return T_B_P*mat*invT_B_P;
////    return invT_B_P*mat;
//}

inline bool isNum(bool b)
{
    if(b!=b) //nan
        return false;
    else if(b==std::numeric_limits<double>::infinity()) //positive inf
        return false;
    else if(b==-std::numeric_limits<double>::infinity()) //positive inf
        return false;
    return true;
}

inline bool isNumf(float b)
{
    if(b!=b) //nan
        return false;
    else if(b==std::numeric_limits<float>::infinity()) //positive inf
        return false;
    else if(b==-std::numeric_limits<float>::infinity()) //positive inf
        return false;
    return true;
}

inline double l2(float *f1,float *f2, int size)
{
    double dist=0;
    double max=std::numeric_limits<double>::max();

    for(int i=0;i<size;i++)
    {
        if(!isNumf(f1[i]))
            std::cout<<"F1 "<<f1[i]<<std::endl;
        if(!isNumf(f2[i]))
            std::cout<<"F2 "<<f2[i]<<std::endl;

        double tmp=sq(f1[i]-f2[i]);
        if(!isNum(tmp))
            std::cout<<"tmp "<<f1[i]<<" "<<f2[i]<<tmp<<std::endl;
        if(max-dist <= tmp )
            return std::numeric_limits<double>::infinity();

        dist+=tmp;
    }

    dist=sqrt(dist);
    return dist;
}

inline float l2(float3 p)
{
    double sum=sq(p.x)+sq(p.y)+sq(p.z);
    
    float ret=(float)sqrt(sum);
    return ret;
}

inline float trace(sMatrix3 m)
{
    float ret=0.0;
    for(int i=0;i<3;i++)
        ret+=m(i,i);

    return ret;
}

inline float3 vec(sMatrix3 M)
{
    float3 v;
    v.x = M(2,1);
    v.y = M(0,2);
    v.z = M(1,0);
    return v;
}


inline float3 logMap(sMatrix3 Rt)
{
    float3 res = make_float3(0,0,0);
    float costheta = (trace(Rt)-1.0)/2.0;
    float theta = acos(costheta);

    if (theta != 0.000)
    {
        sMatrix3 lnR = sMatrix3::zeros();
        lnR =  Rt - transpose(Rt);
        lnR = lnR * ( theta /(2.0*sin(theta)) );
        res = vec(lnR);
    }

    return res;
}

inline float dist(const float3 &p1,const float3 &p2)
{
    float3 p;
    p.x=p1.x-p2.x;
    p.y=p1.y-p2.y;
    p.z=p1.z-p2.z;
    return l2(p);
}

inline __host__  __device__  void eulerFromHomo(const sMatrix4 &pose,float &roll,float &pitch,float &yaw)
{
    roll  = atan2f(pose(2,1), pose(2,2));
    pitch = asinf(-pose(2,0));
    yaw   = atan2f(pose(1,0), pose(0,0));
}
float2 checkPoseErr(sMatrix4 p1,sMatrix4 p2);
#endif // UTILS_H
