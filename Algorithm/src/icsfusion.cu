#include "icsFusion.h"
#include <vector_types.h>
#include "constant_parameters.h"
#include "utils.h"
#include "kernels.h"
#include "volume.h"
#include <thrust/device_vector.h>
#include<stdint.h>
#include<iostream>

#include"constant_parameters.h"

//static bool firstAcquire = true;
dim3 imageBlock = dim3(32, 16);
dim3 raycastBlock = dim3(32, 8);

IcsFusion::IcsFusion(kparams_t par,Matrix4 initPose)
    :params(par),
    _tracked(false)
{
    uint3 vr = make_uint3(params.volume_resolution.x,
                          params.volume_resolution.y,
                          params.volume_resolution.z);

    float3 vd = make_float3(params.volume_size.x,
                            params.volume_size.y,
                            params.volume_size.z);
    volume.init(vr, vd);
    newDataVol.init(vr, vd);

    pose = initPose;
    oldPose=pose;
    this->iterations.clear();
    for (std::vector<int>::iterator it = params.pyramid.begin();it != params.pyramid.end(); it++)
    {    
        this->iterations.push_back(*it);
    }
    largestep=0.75*params.mu;
    inverseCam=getInverseCameraMatrix(params.camera);
    camMatrix=getCameraMatrix(params.camera);
    step = min(params.volume_size) / max(params.volume_resolution);
    viewPose = &pose;

    uint2 cs = make_uint2(params.computationSize.x, params.computationSize.y);
    reduction.alloc(cs);
    vertex.alloc(cs);
    normal.alloc(cs);
    rawDepth.alloc(cs);
    depthImage.alloc(cs);
    rawRgb.alloc(cs);

    scaledDepth.resize(iterations.size());
    inputVertex.resize(iterations.size());
    inputNormal.resize(iterations.size());

    for (int i = 0; i < iterations.size(); ++i)
    {
        scaledDepth[i].alloc(cs >> i);
        inputVertex[i].alloc(cs >> i);
        inputNormal[i].alloc(cs >> i);
    }

    std::cout<<"Size:"<<params.computationSize.x;
    std::cout<<"Size:"<<params.computationSize.y<<std::endl;

    gaussian.alloc(make_uint2(radius * 2 + 1, 1));
    output.alloc(make_uint2(32, 8));
    //generate gaussian array
    generate_gaussian<<< 1,gaussian.size.x>>>(gaussian, delta, radius);
    dim3 grid = divup(dim3(volume.size.x, volume.size.y), imageBlock);
    TICK("initVolume");
    initVolumeKernel<<<grid, imageBlock>>>(volume, make_float2(1.0f, 0.0f));
    TOCK();

    
    // render buffers
    renderModel.alloc(cs);
    //TODO better memory managment of covariance data
    covData.alloc(cs);
    
    if (printCUDAError())
    {
        cudaDeviceReset();
        exit(1);
    }
}

IcsFusion::~IcsFusion()
{
    cudaDeviceSynchronize();
    volume.release();
    
    reduction.release();
    normal.release();
    vertex.release();
    
    for(int i=0;i<inputVertex.size();i++)
    {
        inputVertex[i].release();
    }
    
    for(int i=0;i<inputNormal.size();i++)
    {
        inputNormal[i].release();
    }
     
    for(int i=0;i<scaledDepth.size();i++)
    {
        scaledDepth[i].release();
    }
    
    covData.release();
    rawDepth.release();
    rawRgb.release();
    depthImage.release();
    output.release();
    gaussian.release();
    
    renderModel.release();
    printCUDAError();
}

void IcsFusion::reset()
{
    dim3 grid = divup(dim3(volume.size.x, volume.size.y), imageBlock);
    initVolumeKernel<<<grid, imageBlock>>>(volume, make_float2(1.0f, 0.0f));
}

bool IcsFusion::preprocessing2(const float *inputDepth,const uchar3 *inputRgb)
{
//     uint2 s = make_uint2(params.inputSize.x, params.inputSize.y);
    cudaMemcpy(rawDepth.data(), inputDepth, params.inputSize.x * params.inputSize.y * sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(rawRgb.data(), inputRgb, params.inputSize.x * params.inputSize.y * sizeof(uchar3),cudaMemcpyHostToDevice);

    dim3 grid = divup(make_uint2(params.inputSize.x, params.inputSize.y), imageBlock);
    TICK("bilateral_filter");
    bilateralFilterKernel<<<grid, imageBlock>>>(scaledDepth[0], rawDepth, gaussian, e_delta, radius);
    TOCK();

    return true;
}

bool IcsFusion::preprocessing(const ushort * inputDepth,const uchar3 *inputRgb)
{
    cudaMemcpy(depthImage.data(), inputDepth, params.inputSize.x * params.inputSize.y * sizeof(ushort), cudaMemcpyHostToDevice);
    
    TICK("mm2meters");
    mm2metersKernel<<<divup(rawDepth.size, imageBlock), imageBlock>>>(rawDepth, depthImage);
    TOCK();

    cudaMemcpy(rawRgb.data(), inputRgb, params.inputSize.x * params.inputSize.y * sizeof(uchar3),cudaMemcpyHostToDevice);
    // filter the input depth map
    dim3 grid = divup(make_uint2(params.computationSize.x, params.computationSize.y), imageBlock);
    TICK("bilateral_filter");
    bilateralFilterKernel<<<grid, imageBlock>>>(scaledDepth[0], rawDepth, gaussian, e_delta, radius);
    TOCK();

    return true;
}

bool IcsFusion::tracking(uint frame)
{
    (void)frame;
    forcePose=false;
    std::vector<dim3> grids;
    for (int i = 0; i < iterations.size(); ++i)
        grids.push_back(divup(make_uint2(params.computationSize.x, params.computationSize.y) >> i, imageBlock));

    // half sample the input depth maps into the pyramid levels
    for (int i = 1; i < iterations.size(); ++i)
    {
        TICK("halfSampleRobust");
        halfSampleRobustImageKernel<<<grids[i], imageBlock>>>(scaledDepth[i], scaledDepth[i-1], e_delta * 3, 1);
        TOCK();
    }

    float4 k = make_float4(params.camera.x, params.camera.y, params.camera.z, params.camera.w);
    // prepare the 3D information from the input depth maps
    for (int i = 0; i < iterations.size(); ++i)
    {
        TICK("depth2vertex");
        depth2vertexKernel<<<grids[i], imageBlock>>>( inputVertex[i], scaledDepth[i], getInverseCameraMatrix(k / float(1 << i))); // inverse camera matrix depends on level
        TOCK();
        TICK("vertex2normal");
        vertex2normalKernel<<<grids[i], imageBlock>>>( inputNormal[i], inputVertex[i] );
        TOCK();
    }

    oldPose = pose;
    const Matrix4 projectReference = camMatrix*inverse(Matrix4(&raycastPose));

    for (int level = iterations.size() - 1; level >= 0; --level)
    {
        for (int i = 0; i < iterations[level]; ++i)
        {
            TICK("track");
            trackPose=pose;
            trackKernel<<<grids[level], imageBlock>>>( reduction,
                                                       inputVertex[level],
                                                       inputNormal[level],
                                                       vertex,
                                                       normal,
                                                       Matrix4( & pose ),
                                                       projectReference,
                                                       dist_threshold,
                                                       normal_threshold);
            TOCK();
            TICK("reduce");
            reduceKernel<<<8, 112>>>( output.data(), reduction, inputVertex[level].size ); // compute the linear system to solve
            TOCK();
            cudaDeviceSynchronize();// important due to async nature of kernel call

            TooN::Matrix<8, 32, float, TooN::Reference::RowMajor> values(output.data());
            for(int j = 1; j < 8; ++j)
                values[0] += values[j];

            if (updatePoseKernel(pose, output.data(), params.icp_threshold,this->deltaPose))
                break;
        }
    }
    return checkPoseKernel(pose, oldPose, output.data(), params.computationSize,track_threshold);
}

bool IcsFusion::raycasting(uint frame)
{
    (void)frame;
    bool doRaycast = false;
    if (frame > 2)
    {
        oldRaycastPose = raycastPose;
        raycastPose = pose;
        dim3 grid=divup(make_uint2(params.computationSize.x,params.computationSize.y),raycastBlock );
        TICK("raycast");
        raycastKernel<<<grid, raycastBlock>>>(vertex, normal, volume, Matrix4(&raycastPose) * inverseCam,
                                              nearPlane,
                                              farPlane,
                                              step,
                                              largestep);
        TOCK();
    }

    return doRaycast;
}

void IcsFusion::integrateNewData(sMatrix4 p)
{
    dim3 grid=divup(dim3(newDataVol.size.x, newDataVol.size.y), imageBlock);
    initVolumeKernel<<<grid, imageBlock>>>(newDataVol, make_float2(1.0f, 0.0f));


    integrateKernel<<<grid,imageBlock>>>(newDataVol,rawDepth,rawRgb,
                                         inverse(p),camMatrix,params.mu,maxweight );

}

bool IcsFusion::integration(uint frame)
{
    bool doIntegrate = checkPoseKernel(pose, oldPose, output.data(),params.computationSize, track_threshold);
    if (doIntegrate || frame <= 3)
    {
        TICK("integrate");
        dim3 grid=divup(dim3(volume.size.x, volume.size.y), imageBlock);
        integrateKernel<<<grid, imageBlock>>>(volume,
                                              rawDepth,
                                              rawRgb,
                                              inverse(pose),
                                              camMatrix,
                                              params.mu,
                                              maxweight );
        TOCK();       
        doIntegrate = true;
    }
    else
    {
        doIntegrate = false;
    }

    return doIntegrate;
}

bool IcsFusion::deIntegration(sMatrix4 p,const Host &depth,const Host &rgb)
{
    image_copy(rawDepth,depth, rawDepth.size.x*rawDepth.size.y*sizeof(float));
    image_copy(rawRgb,rgb, rawRgb.size.x*rawRgb.size.y*sizeof(uchar3));

    TICK("deintegrate");
    deIntegrateKernel<<<divup(dim3(volume.size.x, volume.size.y), imageBlock), imageBlock>>>(volume,
                                                                                           rawDepth,
                                                                                           rawRgb,
                                                                                           inverse(Matrix4(&p)),
                                                                                           camMatrix,
                                                                                           params.mu,
                                                                                           maxweight);    
    TOCK();
    return true;
}

bool IcsFusion::reIntegration(sMatrix4 p,const Host &depth,const Host &rgb)
{    
    uint s = params.inputSize.x*params.inputSize.y;
    image_copy(rawDepth,depth, s*sizeof(float));
    image_copy(rawRgb,rgb, s*sizeof(uchar3));
    TICK("reintegrate");
    integrateKernel<<<divup(dim3(volume.size.x, volume.size.y), imageBlock), imageBlock>>>(volume,
                                                                                           rawDepth,
                                                                                           rawRgb,
                                                                                           inverse(Matrix4(&p)),
                                                                                           camMatrix,
                                                                                           params.mu,
                                                                                           maxweight );
    TOCK();
    return true;
}

Image<float3, Host> IcsFusion::getAllVertex()
{
    Image<float3, Host> ret( make_uint2(params.inputSize.x, params.inputSize.y) );
    cudaMemcpy(ret.data(), inputVertex[0].data(),
            params.inputSize.x * params.inputSize.y * sizeof(float3),
            cudaMemcpyDeviceToHost);
    return ret;
}

Image<float3, Host> IcsFusion::getAllNormals()
{
    Image<float3, Host> ret( make_uint2(params.inputSize.x, params.inputSize.y) );
    cudaMemcpy(ret.data(), inputNormal[0].data(),
            params.inputSize.x * params.inputSize.y * sizeof(float3),
            cudaMemcpyDeviceToHost);
    return ret;
}

Image<TrackData, Host> IcsFusion::getTrackData()
{
    Image<TrackData, Host> trackData;
    trackData.alloc(reduction.size);

    cudaMemcpy(trackData.data(), reduction.data(),reduction.size.x*reduction.size.y*sizeof(TrackData),cudaMemcpyDeviceToHost);

    return trackData;
}


void IcsFusion::getVertices(std::vector<float3> &vertices)
{
    vertices.clear();
    short2 *hostData = (short2 *) malloc(volume.size.x * volume.size.y * volume.size.z * sizeof(short2));

    if (cudaMemcpy(hostData,
                   volume.data,
                   volume.size.x *
                   volume.size.y *
                   volume.size.z *
                   sizeof(short2),
                   cudaMemcpyDeviceToHost) != cudaSuccess)
    {
        std::cerr << "Error reading volumetric representation data from the GPU. "<< std::endl;
        exit(1);
    }
    generateTriangles(vertices, volume, hostData);
    free(hostData);
}

void IcsFusion::renderVolume(uchar3 * out)
{
    dim3 grid=divup(renderModel.size,imageBlock);

    TICK("renderVolume");
    renderVolumeKernel2<<<grid,imageBlock>>>( renderModel,vertex,normal,light,ambient,nearPlane,farPlane);
    TOCK();
    

    cudaMemcpy(out, renderModel.data(),
            params.computationSize.x * params.computationSize.y * sizeof(uchar3),
            cudaMemcpyDeviceToHost);
}

Image<float, Host> IcsFusion::vertex2Depth()
{
    Image<float, Host> ret(params.inputSize);
    Image<float, Device> model(params.inputSize);
    
     dim3 grid=divup(model.size,imageBlock);
    vertex2depth<<<grid,imageBlock>>>( model,vertex,normal,nearPlane,farPlane);
    
    cudaMemcpy(ret.data(), model.data(),
            params.inputSize.x * params.inputSize.y * sizeof(float),
            cudaMemcpyDeviceToHost);
    return ret;
}

float IcsFusion::compareRgb( )
{
    Image<float, Device> diff( make_uint2(params.inputSize.x, params.inputSize.y) );
    compareRgbKernel<<<divup(renderModel.size, imageBlock), imageBlock>>>( renderModel,rawRgb,diff);
    
    size_t size=params.inputSize.x*params.inputSize.y;
    thrust::device_ptr<float> diff_ptr(diff.data());
    thrust::device_vector<float> d_vec(diff_ptr,diff_ptr+size);
    float sum = thrust::reduce(d_vec.begin(), d_vec.end(), 0, thrust::plus<float>());

    float ret = sum/size;
    
    diff.release();
    return ret;
}

void IcsFusion::getImageProjection(sMatrix4 p, uchar3 *out)
{

    Image<float3, Device> vertexNew, normalNew;
    vertexNew.alloc(params.inputSize);
    normalNew.alloc(params.inputSize);

    dim3 grid=divup(params.inputSize,raycastBlock );
    //raycast from given pose
    raycastKernel<<<grid, raycastBlock>>>(vertexNew, normalNew, volume, p * inverseCam,
                                         nearPlane,farPlane,step,largestep);
    
    cudaDeviceSynchronize();
    
    grid=divup(params.inputSize,imageBlock );
    renderRgbKernel<<<grid, imageBlock>>>( renderModel,volume,vertexNew,normalNew);

    cudaMemcpy(out, renderModel.data(),
               params.inputSize.x * params.inputSize.y * sizeof(uchar3),
               cudaMemcpyDeviceToHost);
    
    vertexNew.release();
    normalNew.release();
}

float IcsFusion::getWrongNormalsSize()
{
    dim3 grid=divup(make_uint2(params.computationSize.x,params.computationSize.y),raycastBlock );

    Image<int, Device> model;
    model.alloc(params.inputSize);

    wrongNormalsSizeKernel<<<grid, raycastBlock>>>( model,reduction );

    size_t size=params.inputSize.x*params.inputSize.y;

    thrust::device_ptr<int> diff_ptr(model.data());
    thrust::device_vector<int> d_vec(diff_ptr,diff_ptr+size);
    int sum = thrust::reduce(d_vec.begin(), d_vec.end(), 0, thrust::plus<int>());

    float ret = (float)sum/(params.computationSize.x*params.computationSize.y);
    
    model.release();
    return ret;
}

void IcsFusion::renderImage(uchar3 * out)
{
    TICK("renderVolume");
    cudaDeviceSynchronize();
    dim3 grid=divup(renderModel.size, imageBlock);
    renderRgbKernel<<<grid, imageBlock>>>(renderModel,volume,vertex,normal);
    TOCK();

     cudaMemcpy(out, renderModel.data(),
                params.computationSize.x * params.computationSize.y * sizeof(uchar3),
                cudaMemcpyDeviceToHost);

}

void IcsFusion::renderTrack(uchar3 * out)
{
    dim3 grid=divup(renderModel.size, imageBlock);
    TICK("renderTrack");
    renderTrackKernel<<<grid, imageBlock>>>( renderModel, reduction );
    TOCK();
    cudaMemcpy(out, renderModel.data(), params.inputSize.x * params.inputSize.y * sizeof(uchar3), cudaMemcpyDeviceToHost);
}

void IcsFusion::renderDepth(uchar3 * out)
{
    TICK("renderDepthKernel");
    dim3 grid=divup(renderModel.size, imageBlock);
    renderDepthKernel<<<grid, imageBlock>>>( renderModel, rawDepth, nearPlane, farPlane );
    TOCK();
    cudaMemcpy(out,renderModel.data(), params.inputSize.x * params.inputSize.y * sizeof(uchar3), cudaMemcpyDeviceToHost);
}

bool IcsFusion::updatePoseKernel(sMatrix4 & pose, const float * output,float icp_threshold,sMatrix4 &deltaPose)
{

    // Update the pose regarding the tracking result
    TooN::Matrix<8, 32, const float, TooN::Reference::RowMajor> values(output);
    TooN::Vector<6> x = solve(values[0].slice<1, 27>());
    TooN::SE3<> delta(x);
    Matrix4 deltaMat=toMatrix4(delta);
    Matrix4 delta4 = deltaMat * Matrix4(&pose);

    pose.data[0].x = delta4.data[0].x;
    pose.data[0].y = delta4.data[0].y;
    pose.data[0].z = delta4.data[0].z;
    pose.data[0].w = delta4.data[0].w;
    pose.data[1].x = delta4.data[1].x;
    pose.data[1].y = delta4.data[1].y;
    pose.data[1].z = delta4.data[1].z;
    pose.data[1].w = delta4.data[1].w;
    pose.data[2].x = delta4.data[2].x;
    pose.data[2].y = delta4.data[2].y;
    pose.data[2].z = delta4.data[2].z;
    pose.data[2].w = delta4.data[2].w;
    pose.data[3].x = delta4.data[3].x;
    pose.data[3].y = delta4.data[3].y;
    pose.data[3].z = delta4.data[3].z;
    pose.data[3].w = delta4.data[3].w;

    // Return validity test result of the tracking
    if (norm(x) < icp_threshold)
    {
        deltaPose=deltaMat;
        return true;
    }
    return false;
}

bool IcsFusion::checkPoseKernel(sMatrix4 & pose,
                     sMatrix4 oldPose,
                     const float * output,
                     uint2 imageSize,
                     float track_threshold)
{

    if(forcePose)
      return true;
    
    // Check the tracking result, and go back to the previous camera position if necessary
    // return true;
    TooN::Matrix<8, 32, const float, TooN::Reference::RowMajor> values(output);

    if ( (std::sqrt(values(0, 0) / values(0, 28)) > 2e-2) ||
         (values(0, 28) / (imageSize.x * imageSize.y) < track_threshold) )
    {
        pose = oldPose;
        _tracked=false;
        return false;
    }

    _tracked=true;
    return true;
}

void IcsFusion::getImageRaw(Host &to) const
{
  uint s=(uint)rawDepth.size.x*rawDepth.size.y*sizeof(uchar3);
  to.alloc(s);
  cudaMemcpy(to.data, rawRgb.data(),s,cudaMemcpyDeviceToHost);
}

void IcsFusion::getDepthRaw(Host &to) const
{
  uint s=(uint)rawDepth.size.x*rawDepth.size.y*sizeof(float);
  to.alloc(s);
   cudaMemcpy(
     to.data, 
     rawDepth.data(), 
     s, 
     cudaMemcpyDeviceToHost);
//     image_copy( (Host)data,
//     (Device)rawDepth,                
//                  );
}

void IcsFusion::getIcpValues(Image<float3, Host> &depthVertex,
                             Image<float3, Host> &raycastVertex,
                             Image<float3, Host> &raycastNormals,
                             Image<TrackData, Host> &trackData) const
{
    uint s=(uint) (params.volume_size.x*params.volume_size.y);
    depthVertex.alloc(inputVertex[0].size);
    raycastVertex.alloc(vertex.size);
    raycastNormals.alloc(normal.size);
    trackData.alloc(reduction.size);
  
    cudaMemcpy(depthVertex.data(), inputVertex[0].data(),s*sizeof(float3),cudaMemcpyDeviceToHost);
    cudaMemcpy(raycastVertex.data(), vertex.data(),s*sizeof(float3),cudaMemcpyDeviceToHost);
    cudaMemcpy(raycastNormals.data(), normal.data(),s*sizeof(float3),cudaMemcpyDeviceToHost);
    cudaMemcpy(trackData.data(), reduction.data(),reduction.size.x*reduction.size.y*sizeof(TrackData),cudaMemcpyDeviceToHost);
}

sMatrix6 IcsFusion::calculate_ICP_COV()
{
    sMatrix6 ret;
    if(!_tracked)
    {
        for(int i=0;i<6;i++)
            for(int j=0;j<6;j++)
                ret(i,j)=cov_big;

        return ret;
    }


    sMatrix4 currPose=pose;
    sMatrix4 invPrevPose=inverse(oldPose);
    sMatrix4 delta=invPrevPose*currPose;

    Matrix4 projectedReference = camMatrix*inverse(Matrix4(&raycastPose));

//     sMatrix4 delta=inverse(this->getPose())*this->getDeltaPose()*this->getPose();
//     sMatrix4 delta=this->getDeltaPose();
    
//     Matrix4 pose=this->getDeltaPose();
//     pose=inverse(pose)*this->getPose();
//     pose=inverse(pose)*this->getPose();
//    sMatrix4 delta=fromVisionCord(this->getDeltaPose());
    
//     sMatrix4 T_B_P;
//     T_B_P.data[0]=make_float4(0,-1,0,0);
//     T_B_P.data[1]=make_float4(0,0,-1,0);
//     T_B_P.data[2]=make_float4(1,0,0,0);
//     T_B_P.data[3]=make_float4(0,0,0,1);
//     delta=T_B_P*delta;
//     
    dim3 grid=divup(make_uint2(params.inputSize.x,params.inputSize.y),imageBlock );
    sMatrix6 initMat;
    for(int i=0;i<36;i++)
        initMat.data[i]=0.0;
    
    delta=fromVisionCord(delta);
    icpCovarianceFirstTerm<<<grid, imageBlock>>>(inputVertex[0],
                                                vertex,
                                                normal,
                                                reduction,
                                                covData,
                                                trackPose,
                                                projectedReference,
                                                invPrevPose,
                                                delta);
    
    cudaDeviceSynchronize();    
    
    size_t size=covData.size.x*covData.size.y;
    thrust::device_ptr<sMatrix6> cov_ptr(covData.data());

    Image<sMatrix6, Host> covDataHost(covData.size);
    covDataHost.alloc(covData.size);

    cudaMemcpy(covDataHost.data(), covData.data(),size*sizeof(sMatrix6),cudaMemcpyDeviceToHost);
    sMatrix6 sum;
    for(int i=0;i<36;i++)
        sum.data[i]=0.0;
    uint2 idx;
    for(idx.x=0;idx.x<covData.size.x;idx.x++)
    {
        for(idx.y=0;idx.y<covData.size.y;idx.y++)
        {
            sum=sum+covDataHost[idx];
        }
    }

//    covDataHost.release();

    thrust::device_vector<sMatrix6> d_vec(cov_ptr,cov_ptr+size);

    
    sMatrix6 d2J_dX2 = thrust::reduce(d_vec.begin(), d_vec.end(), initMat, thrust::plus<sMatrix6>());
    cudaDeviceSynchronize();

    /*
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
    std::cout<<sum<<std::endl;
    std::cout<<d2J_dX2<<std::endl;
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
    */
    d2J_dX2=sum;

    icpCovarianceSecondTerm<<<grid, imageBlock>>>(inputVertex[0],
                                                  vertex,
                                                  normal,
                                                  reduction,
                                                  covData,
                                                  trackPose,
                                                  projectedReference,
                                                  delta,
                                                  invPrevPose,
                                                  1.0);
    cudaDeviceSynchronize();

    thrust::device_ptr<sMatrix6> cov_ptr2(covData.data());
    thrust::device_vector<sMatrix6> d_vec2(cov_ptr2,cov_ptr2+size);

    sMatrix6 mat = thrust::reduce(d_vec2.begin(), d_vec2.end(), initMat, thrust::plus<sMatrix6>());
    //ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();


    cudaMemcpy(covDataHost.data(), covData.data(),size*sizeof(sMatrix6),cudaMemcpyDeviceToHost);
    for(int i=0;i<36;i++)
        sum.data[i]=0.0;

    for(idx.x=0;idx.x<covData.size.x;idx.x++)
    {
        for(idx.y=0;idx.y<covData.size.y;idx.y++)
        {
            sum=sum+covDataHost[idx];
        }
    }

    covDataHost.release();

    cudaDeviceSynchronize();

    /*
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
    std::cout<<sum<<std::endl;
    std::cout<<mat<<std::endl;
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
    */
    mat=sum;


    sMatrix6 d2J_dX2inv=inverse(d2J_dX2);
    sMatrix6 tmp=d2J_dX2inv * mat;
    sMatrix6 ICP_COV= tmp * d2J_dX2inv;

    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
            float val;
            //eliminate NaN values
            if(ICP_COV(i,j)!=ICP_COV(i,j) || ICP_COV(j,i)!=ICP_COV(j,i))
            {
                val=cov_big;
            }
            else
            {
                val=( ICP_COV(i,j) + ICP_COV(j,i) ) / 2;
                if( val!=val || std::isinf(val) || val > cov_big )
                {
//                    val=COV_BIG;
                }
            }
            ret(i,j)=val;
            ret(j,i)=val;
        }
    }

//    std::cout<<"======================="<<std::endl;
//    std::cout<<d2J_dX2<<std::endl;
//    std::cout<<d2J_dX2inv<<std::endl;
//    std::cout<<mat<<std::endl;
//    std::cout<<tmp<<std::endl;
//    std::cout<<ICP_COV<<std::endl;
//    std::cout<<ret<<std::endl;
//    std::cout<<"F:"<<FLT_MAX<<std::endl;
//    std::cout<<"======================="<<std::endl;


//    std<<cout<<"======================="<<std::endl;

    //ICP_COV=ICP_COV;
    return ret;
}



