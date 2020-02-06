#include"kernelscalls.h"
#include"kernels.h"

sMatrix6 calculatePoint2PointCov(float3 *vert,
                                 int vertSize,
                                 float3 *prevVert,
                                 int prevVertSize,
                                 int2 *corresp,
                                 int correspSize,
                                 sMatrix4 tf)
{
    sMatrix6 ret;

    float3 *vertGpu;
    cudaMalloc(&vertGpu,vertSize*sizeof(float3));
    cudaMemcpy(vertGpu,vert,vertSize*sizeof(float3),cudaMemcpyHostToDevice);

    float3 *prevVertGpu;
    cudaMalloc(&prevVertGpu,prevVertSize*sizeof(float3));
    cudaMemcpy(prevVertGpu,prevVert,prevVertSize*sizeof(float3),cudaMemcpyHostToDevice);

    int2 *correspGpu;
    cudaMalloc(&correspGpu,correspSize*sizeof(int2));
    cudaMemcpy(correspGpu,corresp,correspSize*sizeof(int2),cudaMemcpyHostToDevice);

//    sMatrix6 covData;
//    point2PointCovFirstTerm<<<grid, imageBlock>>>(vertGpu,
//                                                  vertSize,
//                                                  prevVertGpu,
//                                                  prevVertSize,
//                                                  corresp,
//                                                  correspSize,
//                                                  tf,
//                                                  covData);


    cudaFree(vertGpu);
    cudaFree(prevVertGpu);
    cudaFree(correspGpu);

    return ret;


    /*
    sMatrix4 currPose=pose;
    sMatrix4 invPrevPose=inverse(oldPose);
    sMatrix4 delta=invPrevPose*currPose;

    Matrix4 projectedReference = camMatrix*inverse(Matrix4(&raycastPose));
    dim3 grid=divup(make_uint2(params.inputSize.x,params.inputSize.y),imageBlock );

    sMatrix6 initMat;
    for(int i=0;i<36;i++)
        initMat.data[i]=0.0;


    icpCovarianceFirstTerm<<<grid, imageBlock>>>(inputVertex[0],
                                                vertex,
                                                normal,
                                                reduction,
                                                covData,
                                                trackPose,
                                                projectedReference,
                                                delta);

    cudaDeviceSynchronize();
    size_t size=covData.size.x*covData.size.y;
    thrust::device_ptr<sMatrix6> cov_ptr(covData.data());
    sMatrix6 d2J_dX2 = thrust::reduce(cov_ptr, cov_ptr+size, initMat, thrust::plus<sMatrix6>());

    icpCovarianceSecondTerm<<<grid, imageBlock>>>(inputVertex[0],
                                                  vertex,
                                                  normal,
                                                  reduction,
                                                  covData,
                                                  trackPose,
                                                  projectedReference,
                                                  delta,
                                                  1.0);
    cudaDeviceSynchronize();
    sMatrix6 covSecondTerm = thrust::reduce(cov_ptr, cov_ptr+size, initMat, thrust::plus<sMatrix6>());


    sMatrix6 d2J_dX2inv=inverse(d2J_dX2);
    sMatrix6 tmp=d2J_dX2inv * covSecondTerm;
    sMatrix6 icpCov= tmp * d2J_dX2inv;

    //make sure that covariance matrix is symetric.
    //small asymetries may occur due to numerical stability
    sMatrix6 ret;
    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
            //eliminate NaN values
            if(icpCov(i,j)!=icpCov(i,j))
            {
                icpCov(i,j)=cov_big;
            }
            if(icpCov(j,i)!=icpCov(j,i))
            {
                icpCov(j,i)=cov_big;
            }
            float val=( icpCov(i,j) + icpCov(j,i))/2;
            ret(i,j)=val;
            ret(j,i)=val;

        }
    }
    */

}
