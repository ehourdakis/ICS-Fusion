#include"utils.h"
#include <Eigen/Dense>

void calculate_ICP_COV(const Image<float3, Host> &dataVertex, 
                       const Image<float3, Host> &modelVertex, 
                       const Image<float3, Host> &modelNormals,
                       Image<TrackData,Host> &trackData,
                       Eigen::Matrix4f& transform, 
                       Eigen::MatrixXd& ICP_COV);
