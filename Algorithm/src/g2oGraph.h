#ifndef G2O_GRAPH_H 
#define G2O_GRAPH_H

#include"PoseGraph.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include<g2o/types/slam3d/edge_se3.h>

#include<vector>
#include"kparams.h"

class G2oGraph :public PoseGraph
{
    public:
        G2oGraph(kparams_t params);
        virtual ~G2oGraph()
        {
            clear();
        }
        virtual void init(const sMatrix4 &initialPose) override;
        virtual void addFrame(const sMatrix4 &pose,const sMatrix6 &cov) override;
        virtual int addLandmark(float3 pos) override;
        virtual void connectLandmark(float3 pos,int landIdx,int poseIdx, sMatrix3 &cov) override;
        virtual void connectLandmark(float2 pos, int lid, double Sigma2);
        virtual void clear() override;
        virtual double optimize(int frame) override;
        
        virtual sMatrix4 getPose(int i)  override;
        virtual uint poseSize() const override;
        virtual uint landmarksSize() const override;
    
    private:
        g2o::SparseOptimizer optimizer;
        
        bool _isDense;
        bool bRobust;
        uint poseId;
        uint landmarkId;
        uint odomId;
        kparams_t _params;
        int nIterations;

        sMatrix4 prevPose;
        
        std::vector<g2o::VertexSE3 *> vertexes;
        g2o::VertexSE3 *prevVertex;
        
        static g2o::SE3Quat toG2oPose(const sMatrix4 &pose);
        static g2o::Isometry3 toG2oIsometry(const sMatrix4 &pose);
        static Eigen::Matrix<double,3,1> toG2oPoint(const float3 &vec);
        static sMatrix4 fromG2oPose(const g2o::SE3Quat &g2oPose);


};
#endif
