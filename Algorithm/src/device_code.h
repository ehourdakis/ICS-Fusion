#ifndef DEVICE_CODE_H
#define DEVICE_CODE_H

//__forceinline__ __device__ float sq(const float x)
//{
//    return x * x;
//}

#if 1
__device__ __forceinline__ float4 raycast(const Volume volume,
                                          const uint2 pos,
                                          const Matrix4 view,
                                          const float nearPlane,
                                          const float farPlane,
                                          const float step,
                                          const float largestep,int frame)
{   
    const float3 origin = view.get_translation();
    float3 pix;
    pix.x=pos.x+volume.getOffset().x;
    pix.y=pos.y+volume.getOffset().y;
    pix.z=volume.getOffset().z;
    const float3 direction = rotate(view, make_float3(pos.x, pos.y, 1.f));
    const float3 invR = make_float3(1.0f) / direction;
    //const float3 direction = rotate(view, make_float3(pix.x, pix.y, 1.f));

    /*
    // intersect ray with a box
    // http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
    // compute intersection of ray with all six bbox planes
    const float3 invR = make_float3(1.0f) / direction;
    const float3 tbot = -1 * invR * origin;
//    const float3 ttop = invR * (volume.getDimensions() - origin);
    const float3 ttop = invR * (volume.getDimWithOffset() - origin);
    // re-order intersections to find smallest and largest on each axis
    const float3 tmin = fminf(ttop, tbot);
    const float3 tmax = fmaxf(ttop, tbot);
    */

    float3 tmin;
    float3 tmax;

    if (invR.x >= 0)
    {
        tmin.x=(volume.getOffsetPos().x-origin.x)*invR.x;
        tmax.x=(volume.getDimWithOffset().x-origin.x)*invR.x;
        //tmin = (min.x - r.orig.x) * invdir.x;
        //tmax = (max.x - r.orig.x) * invdir.x;
    }
    else
    {
        tmin.x=(volume.getDimWithOffset().x-origin.x)*invR.x;
        tmax.x=(volume.getOffsetPos().x-origin.x)*invR.x;
        //tmin = (max.x - r.orig.x) * invdir.x;
        //tmax = (min.x - r.orig.x) * invdir.x;
    }

    if (invR.y >= 0)
    {
        tmin.y=invR.y*(volume.getOffsetPos().y-origin.y);
        tmax.y=invR.y*(volume.getDimWithOffset().y-origin.y);
        //tmin = (min.x - r.orig.x) * invdir.x;
        //tmax = (max.x - r.orig.x) * invdir.x;
    }
    else
    {
        tmin.y=invR.y*(volume.getDimWithOffset().y-origin.y);
        tmax.y=invR.y*(volume.getOffsetPos().y-origin.y);
        //tmin = (max.x - r.orig.x) * invdir.x;
        //tmax = (min.x - r.orig.x) * invdir.x;
    }

    if (invR.z >= 0)
    {
        tmin.z=invR.z*(volume.getOffsetPos().z-origin.z);
        tmax.z=invR.z*(volume.getDimWithOffset().z-origin.z);
        //tmin = (min.x - r.orig.x) * invdir.x;
        //tmax = (max.x - r.orig.x) * invdir.x;
    }
    else
    {
        tmin.z=invR.z*(volume.getDimWithOffset().z-origin.z);
        tmax.z=invR.z*(volume.getOffsetPos().z-origin.z);
        //tmin = (max.x - r.orig.x) * invdir.x;
        //tmax = (min.x - r.orig.x) * invdir.x;
    }

    if(frame==299)
    {
        printf("%f %f %f\n",invR.x,invR.y,invR.z);
    }


    // find the largest tmin and the smallest tmax
    const float largest_tmin = fmaxf(fmaxf(tmin.x, tmin.y),
                                     fmaxf(tmin.x, tmin.z));
    const float smallest_tmax = fminf(fminf(tmax.x, tmax.y),
                                      fminf(tmax.x, tmax.z));

    // check against near and far plane
//    printf("largest_tmin:%f\n",largest_tmin);
//    printf("smallest_tmax:%f\n",smallest_tmax);

    float tnear;
    float tfar;
//    if(largest_tmin<0)
//    {
//        float tnear = fmaxf(largest_tmin, -nearPlane);

////        printf("tnear:%f tfar:%f\n",tnear,tfar);
//    }
//    else
//    {
//        float tnear = fmaxf(largest_tmin, nearPlane);

//    }

//    if(smallest_tmax<0)
//    {
//        float tfar = fminf(smallest_tmax, -farPlane);
//    }
//    else
//    {
//        float tfar = fminf(smallest_tmax, farPlane);
//    }

//    tnear=largest_tmin;
//    tfar=smallest_tmax;

    // check against near and far plane
    tnear = fmaxf(largest_tmin, nearPlane);
    tfar = fminf(smallest_tmax, farPlane);

//    int tnear_signe= (largest_tmin<0)?-1:1;
//    int tfar_signe= (largest_tmin<0)?-1:1;



    if(frame==4)
    {
//        printf("%f %f %f\n",invR.x,invR.y,invR.z);
        printf("tnear:%f %f %f %f\n",tnear,largest_tmin,tfar,smallest_tmax);
//        printf("tfar:%f %f\n",tfar,smallest_tmax);
    }

//    int signe;
//    if(tfar<0)
//        signe=-1;
//    else
//        signe=1;
    //const float tnear =nearPlane;
    //const float tfar =farPlane;


    if (tnear < tfar)
    {
        // first walk with largesteps until we found a hit
        float t = tnear;
        float stepsize = largestep;
        float f_t = volume.interp(origin + direction * t);
        float f_tt = 0;

        if (f_t > 0) // ups, if we were already in it, then don't render anything here
        {
            for (; t < tfar; t += stepsize)
            {
                f_tt = volume.interp(origin + direction * t);
                if (f_tt < 0)                  // got it, jump out of inner loop
                    break;
                if (f_tt < 0.8f)               // coming closer, reduce stepsize
                    stepsize = step;
                f_t = f_tt;
            }
            if (f_tt < 0)
            {           // got it, calculate accurate intersection
                t = t + stepsize * f_tt / (f_t - f_tt);
                return make_float4(origin + direction * t, t);
            }
        }
    }
    return make_float4(0);
}
#else
__device__ __forceinline__ float4 raycast(const Volume volume,
                                          const uint2 pos,
                                          const Matrix4 view,
                                          const float nearPlane,
                                          const float farPlane,
                                          const float step,
                                          const float largestep,
                                          int frame)
{
    const float3 origin = view.get_translation();
    const float3 direction = rotate(view, make_float3(pos.x, pos.y, 1.f));

    // intersect ray with a box
    // http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
    // compute intersection of ray with all six bbox planes
    const float3 invR = make_float3(1.0f) / direction;
    const float3 tbot = -1 * invR * origin;
    const float3 ttop = invR * (volume.getDimWithOffset() - origin);

    // re-order intersections to find smallest and largest on each axis
    const float3 tmin = fminf(ttop, tbot);
    const float3 tmax = fmaxf(ttop, tbot);

    // find the largest tmin and the smallest tmax
    const float largest_tmin = fmaxf(fmaxf(tmin.x, tmin.y),
                                     fmaxf(tmin.x, tmin.z));
    const float smallest_tmax = fminf(fminf(tmax.x, tmax.y),
                                      fminf(tmax.x, tmax.z));

    // check against near and far plane
    const float tnear = fmaxf(largest_tmin, nearPlane);
    const float tfar = fminf(smallest_tmax, farPlane);

    if(frame==4)
    {
//        printf("%f %f %f\n",invR.x,invR.y,invR.z);
        printf("tnear:%f %f %f %f\n",tnear,largest_tmin,tfar,smallest_tmax);
//        printf("tfar:%f %f\n",tfar,smallest_tmax);
    }


    if (tnear < tfar)
    {
        // first walk with largesteps until we found a hit
        float t = tnear;
        float stepsize = largestep;
        float f_t = volume.interp(origin + direction * t);
        float f_tt = 0;

        if (f_t > 0) // ups, if we were already in it, then don't render anything here
        {
            for (; t < tfar; t += stepsize)
            {
                f_tt = volume.interp(origin + direction * t);
                if (f_tt < 0)                  // got it, jump out of inner loop
                    break;
                if (f_tt < 0.8f)               // coming closer, reduce stepsize
                    stepsize = step;
                f_t = f_tt;
            }
            if (f_tt < 0)
            {           // got it, calculate accurate intersection
                t = t + stepsize * f_tt / (f_t - f_tt);
                return make_float4(origin + direction * t, t);
            }
        }
    }
    return make_float4(0);
}
#endif


#endif // DEVICE_CODE_H
