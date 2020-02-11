#include<Isam.h>
#include<constant_parameters.h>


using namespace isam;

namespace isam
{
    const int Pose3d::dim;
}


Isam::Isam()
{
}

void Isam::addFrame(const sMatrix4 &pose,const sMatrix6 &covar)
{
    sMatrix6 cov=covar;
    sMatrix4 delta=inverse(prevPose)*pose;
    prevPose=pose;

    Eigen::MatrixXd eigenCov=toEigen(cov);
    Noise noise = isam::Covariance(eigenCov);

    Pose3d vo=toIsamPose(delta);
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    slam->add_node(new_pose_node);
    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes.back(),
                                                            new_pose_node, vo, noise);

    pose_nodes.push_back(new_pose_node);

    factors.push_back(factor);
    slam->add_factor(factor);
}

int Isam::addLandmark(float3 pos)
{
    (void)pos;
    int ret=landmarks.size();

    Point3d_Node *landmark=new Point3d_Node();
    landmarks.push_back(landmark);

    slam->add_node(landmark);

    return ret;
}

void Isam::connectLandmark(float3 pos,int landIdx,int poseIdx, sMatrix3 &cov)
{
    if(poseIdx<0)
        poseIdx=poseSize()+poseIdx;
    
//    pos.x*=-1;
//    pos.y*=-1;
//    pos.z*=-1;
    Point3d point=toIsamPoint(pos);



//     cov=sMatrix3();
//     cov=cov*0.1;
    
    Eigen::MatrixXd eigenCov=toEigen(cov);
    Noise noise = isam::Covariance(eigenCov);

    Pose3d_Point3d_Factor* f=new Pose3d_Point3d_Factor(
                                           pose_nodes[poseIdx],landmarks[landIdx],point,noise);

    factors.push_back(f);
    slam->add_factor(f); 
}

void Isam::addFixPose(const sMatrix4 &fixPose)
{
    sMatrix6 cov;
    cov=cov*cov_small;
    Eigen::MatrixXd eigenCov=toEigen(cov);

    sMatrix4 delta=fixPose;
    
    Noise noise = isam::Covariance(eigenCov);
    Pose3d vo= toIsamPose(delta);
    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes.front(),
                                                            pose_nodes.back(), vo, noise);
    
    factors.push_back(factor);

    slam->add_factor(factor);
}

void Isam::addPoseConstrain(int p1,int p2,const sMatrix4 &delta, const sMatrix6 &cov)
{
    Eigen::MatrixXd eigenCov=toEigen(cov);

    Noise noise = isam::Covariance(eigenCov);
    Pose3d vo= toIsamPose(delta);

    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes[p1],
                                                            pose_nodes[p2],
                                                            vo,
                                                            noise);


    factors.push_back(factor);

    slam->add_factor(factor);
}

double Isam::optimize(int frame)
{
    //std::cout<<"Start isam optimization"<<std::endl;
    
    //char buf[32];
    //sprintf(buf,"data/isam/f_%d_graph",frame);
    //slam->save(buf);

    slam->batch_optimization();


    //sprintf(buf,"data/isam/f_%d_graph_opt",frame);
    //slam->save(buf);

    //slam->print_stats();
    
    double error=slam->chi2();    

    //std::cout<<"End isam optimization"<<std::endl;
    return error;
}

void Isam::init(const sMatrix4 &initalP)
{
    sMatrix4 initalPose=initalP;
    slam=new Slam();
    prevPose=initalP;

    //Disable isam prints
    isam::Properties props=slam->properties();
    props.max_iterations=50;
    props.quiet=true;
    //props.method = DOG_LEG;
    props.method=LEVENBERG_MARQUARDT;
    slam->set_properties(props);

    Pose3d_Node *initial_pose_node = new Pose3d_Node();
    pose_nodes.push_back(initial_pose_node);
    slam->add_node(initial_pose_node);



    Pose3d origin=toIsamPose(initalPose);
    Noise noise =  isam::Covariance(Eigen::MatrixXd::Identity(6, 6)* cov_small );

    Pose3d_Factor* prior = new Pose3d_Factor(initial_pose_node, origin, noise);
    factors.push_back(prior);
    slam->add_factor(prior);

//    Point3d_Node *landmark=new Point3d_Node();
//    landmarks.push_back(landmark);
//    slam->add_node(landmark);

//    Noise noise2 = isam::Covariance(Eigen::MatrixXd::Identity(3, 3) * SMALL_COV );
//    sMatrix4 delta1=inverse(initalPose);


//    float3 pos1=make_float3(delta1(0,3),delta1(1,3),delta1(2,3));
//    Point3d p1=toIsamPoint(pos1);
//    Pose3d_Point3d_Factor* f1=new Pose3d_Point3d_Factor(
//                                           initial_pose_node,landmark,p1,noise2);

//    slam->add_factor(f1);
}

sMatrix4 Isam::getPose(int i) 
{
    return fromIsamNode(pose_nodes[i]);
}

void Isam::clear()
{
//    slam->save("isam.graph");
    if(slam!=nullptr)
    {
        delete slam;
        slam=nullptr;
    }

    for(uint i=0; i<pose_nodes.size();i++ )
    {
        Pose3d_Node *node=pose_nodes[i];
        delete node;
    }

    for(uint i=0; i<factors.size();i++ )
    {
        Factor *f=factors[i];
        delete f;
    }

    for(uint i=0; i<landmarks.size();i++ )
    {
        isam::Point3d_Node *l=landmarks[i];
        delete l;
    }

    pose_nodes.clear();
    factors.clear();
    landmarks.clear();
}

Point3d Isam::toIsamPoint(const float3 &f)
{
//    float3 p=fromVisionCordV(f);
    float3 p=f;
    return Point3d(p.x,p.y,p.z);
}

Pose3d Isam::toIsamPose(const sMatrix4 &pose)
{
//     sMatrix4 pose_trans=fromVisionCord(pose);
    sMatrix4 pose_trans=pose;
    Eigen::Matrix4f eigen_p;
    Eigen::Matrix3f rot;
    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            eigen_p(i,j)=pose_trans(i,j);
        }
    }

    isam::Pose3d delta(eigen_p.cast<double>() );
    return delta;
}

sMatrix4 Isam::fromIsamNode(Pose3d_Node *node)
{
    sMatrix4 ret;
    Eigen::Matrix4d eigen_p=node->value().wTo();

    
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j) = eigen_p(i,j);

//     ret=toVisionCord(ret);
    return ret;
}

Eigen::MatrixXd Isam::toEigen(sMatrix4 mat)
{
    Eigen::MatrixXd ret(4,4);
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            ret(i,j)=mat(i,j);
        }
    }
    return ret;
}

Eigen::MatrixXd Isam::toEigen(sMatrix3 mat)
{
    Eigen::MatrixXd ret(3,3);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            ret(i,j)=mat(i,j);
        }
    }
    return ret;
}

Eigen::MatrixXd Isam::toEigen(sMatrix6 mat)
{
    Eigen::MatrixXd ret(6,6);
    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
            ret(i,j)=mat(i,j);
        }
    }
    return ret;
}
/*
mrpt::poses::CPose3DPDFGaussian Isam::toCPose3DPDF(sMatrix4 pose,sMatrix6 cov)
{
    mrpt::poses::CPose3DPDFGaussian A(mrpt::poses::UNINITIALIZED_POSE);

    A.mean=toCPose(pose);

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            A.cov(i,j)=cov(i,j);

    return A;
}

mrpt::poses::CPose3D Isam::toCPose(sMatrix4 pose)
{
//     std::cout<<"EDO"<<std::endl;
    mrpt::math::CMatrixDouble mat(4,4);
    
//     Eigen::Matrix3f rot;
    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
//             rot(i,j)=pose(i,j);
            mat(i,j)=pose(i,j);
        }
    }
//     Eigen::Vector3f rotV = rot.eulerAngles(0, 1, 2);

//     mrpt::poses::CPose3D ret(pose(0,3),pose(1,3),pose(2,3),
//                              rotV[2],rotV[1],rotV[0]);
// std::cout<<"EDO2"<<std::endl;
    mrpt::poses::CPose3D ret(mat);
    
    return ret;

}
*/
