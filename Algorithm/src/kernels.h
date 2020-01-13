#ifndef KERNEL_GLOBALS_H
#define KERNEL_GLOBALS_H

#include "utils.h"
#include"volume.h"

__global__ void initVolume(Volume volume, const float2 val);

__global__ void integrate(Volume vol,
                          const Image<float> depth,
                          const Matrix4 view,
                          const float mu,
                          const float maxweight);


__global__ void depth2vertex(Image<float3> vertex,
                             const Image<float> depth,
                             const Matrix4 invK);

__global__ void vertex2normal(Image<float3> normal,
                              const Image<float3> vertex);

__global__ void vertex2depth(Image<float> render,
                                   Image<float3> vertex,
                                   Image<float3> normal,
                                   const float nearPlane,
                                   const float farPlane);

__global__ void bilateral_filter(Image<float> out,
                                 const Image<float> in,
                                 const Image<float> gaussian,
                                 float e_d,
                                 int r);

__global__ void track(Image<TrackData> output,
                      const Image<float3> inVertex,
                      const Image<float3> inNormal,
                      const Image<float3> refVertex,
                      const Image<float3> refNormal,
                      const Matrix4 Ttrack,
                      const Matrix4 view,
                      const float dist_threshold,
                      const float normal_threshold);

__global__ void reduce(float * out, const Image<TrackData> J, const uint2 size);

__global__ void trackAndReduce(float * out,
                               const Image<float3> inVertex,
                               const Image<float3> inNormal,
                               const Image<float3> refVertex,
                               const Image<float3> refNormal,
                               const Matrix4 Ttrack,
                               const Matrix4 view,
                               const float dist_threshold,
                               const float normal_threshold);

__global__ void wrongNormalsSizeKernel(Image<int> out,const Image<TrackData> data);

__global__ void compareVertexKernel(Image<float3> vertex1,
                                    Image<float3> vertex2,
                                    Image<float>out);

__global__ void renderRgbKernel(Image<uchar3> render,
                                const Volume volume,
                                Image<float3> vert,
                                Image<float3> norm);

__global__ void generate_gaussian(Image<float> out, float delta, int radius);
__global__ void initVolumeKernel(Volume volume, const float2 val);
__global__ void clearVolumeZ(Volume volume, const float2 val, const int zz, Volume slice);
__global__ void clearVolumeX(Volume volume, const float2 val, const int xx, Volume slice);
__global__ void clearVolumeY(Volume volume, const float2 val, const int yy, Volume slice);

__global__ void bilateralFilterKernel(Image<float> out,
                                      const Image<float> in,
                                      const Image<float> gaussian,
                                      const float e_d,
                                      const int r);

__global__ void mm2metersKernel( Image<float> depth, const Image<ushort> in );
__global__ void halfSampleRobustImageKernel(Image<float> out,
                                            const Image<float> in,
                                            const float e_d,
                                            const int r);

__global__ void depth2vertexKernel(Image<float3> vertex,const Image<float> depth, const Matrix4 invK);
__global__ void vertex2normalKernel(Image<float3> normal,const Image<float3> vertex);
__global__ void reduceKernel(float * out, const Image<TrackData> J,const uint2 size);
__global__ void trackKernel(Image<TrackData> output,
                            const Image<float3> inVertex,
                            const Image<float3> inNormal,
                            const Image<float3> refVertex,
                            const Image<float3> refNormal,
                            const Matrix4 Ttrack,
                            const Matrix4 view,
                            const float dist_threshold,
                            const float normal_threshold);

__global__ void raycastKernel(Image<float3> pos3D,
                              Image<float3> normal,
                              const Volume volume,
                              const Matrix4 view,
                              const float nearPlane,
                              const float farPlane,
                              const float step,
                              const float largestep, int frame);

__global__ void integrateKernel(Volume vol, const Image<float> depth,
                                const Image<uchar3> rgb,
                                const Matrix4 invTrack,
                                const Matrix4 K,
                                const float mu,
                                const float maxweight);

__global__ void deIntegrateKernel(Volume vol,
                                  const Image<float> depth,
                                  const Image<uchar3> rgb,
                                  const Matrix4 invTrack,
                                  const Matrix4 K,
                                  const float mu,
                                  const float maxweight);

__global__ void compareRgbKernel(const Image<uchar3> image1,
                                 const Image<uchar3> image2,
                                 Image<float>out);
__global__ void renderTrackKernel(Image<uchar3> out,const Image<TrackData> data);
__global__ void renderDepthKernel(Image<uchar3> out,
                                  const Image<float> depth,
                                  const float nearPlane,
                                  const float farPlane);
__global__ void renderVolumeKernel(Image<uchar3> render,
                                   Image<float3> vertex,
                                   Image<float3> normal,
                                   const float3 light,
                                   const float3 ambient);
__global__ void copyVolumeData(uint3 *posArray, const Volume volume, float *dest);
__global__ void copyVolumeData2(uint3 *posArray, const Volume volume,
                                float *dest, char *isEmpty);

__global__ void renderVolumeKernel2(Image<uchar3> render,
                                   Image<float3> vertex,
                                   Image<float3> normal,
                                   const float3 light,
                                   const float3 ambient,
                                   const float nearPlane,
                                   const float farPlane);


//=================ICP COVARIANCE======================


__global__ void icpCovarianceFirstTerm(const Image<float3, Device> dataVertex,
                                        const Image<float3, Device> modelVertex,
                                        const Image<float3, Device> modelNormals,
                                        const Image<TrackData, Device> trackData,
                                        Image<sMatrix6, Device> outData,
                                        const Matrix4 pose,
                                        const Matrix4 view,
                                        const Matrix4 delta,
                                        const Matrix4 invPrevPose);


__global__ void icpCovarianceSecondTerm(const Image<float3, Device>  dataVertex,
                                         const Image<float3, Device> modelVertex,
                                         const Image<float3, Device> modelNormals,
                                         const Image<TrackData, Device>  trackData,
                                         Image<sMatrix6, Device> outData,
                                         const Matrix4 Ttrack,
                                         const Matrix4 view,
                                         const Matrix4 delta,
                                         const Matrix4 invPrevPose,
                                         float cov_z);
#endif // KERNEL_GLOBALS_H
