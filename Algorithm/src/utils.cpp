#include"utils.h"

struct timespec tick_clockData;
struct timespec tock_clockData;
bool print_kernel_timing = false;

sMatrix4 T_B_P(0,-1,  0 ,0,
               0, 0, -1, 0,
               1, 0,  0, 0,
               0, 0,  0, 1 );

sMatrix4 invT_B_P=inverse(T_B_P);

float2 checkPoseErr(sMatrix4 p1,sMatrix4 p2)
{
    float2 ret;
    sMatrix3 r1,r2;

    float3 tr1=make_float3(p1(0,3),p1(1,3),p1(2,3));
    float3 tr2=make_float3(p2(0,3),p2(1,3),p2(2,3));

    tr1=tr1-tr2;
    ret.x=l2(tr1);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            r1(i,j)=p1(i,j);
            r2(i,j)=p2(i,j);
        }
    }
    r1=r1*transpose(r2);
    float3 f=logMap(r1);

    ret.y=l2(f);
    return ret;
}
