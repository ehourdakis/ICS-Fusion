#include<vector>
#include<random>

#include<Eigen/StdVector>
#include<fstream>


#include<isam/slam3d.h>
#include<isam/Properties.h>
#include<isam/isam.h>
#include<isam/Point3d.h>
#include<isam/Pose3d.h>

#define NN 21
#define NL 15
#define SKIP_CON 5

// #define ADD_NOISE

#define SMALL_COV 1e-6
// #define SMALL_COV 1e-15

// #define fabs
using namespace Eigen;

struct PoseType
{
    double x;
    double y;
    double z;
    
    double yaw;
    double pitch;
    double roll;
};

const int isam::Pose3d::dim;
std::vector<isam::Pose3d> poses;
std::vector<PoseType> truePoses;
std::vector<isam::Point3d_Node*> landmarks;


void savePoses(char *fileName,std::vector<PoseType> &poses)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    Eigen::Matrix3f rot;
    for(uint i=0;i<poses.size();i++)
    {
        file<<poses[i].x<<','<<poses[i].y<<','<<poses[i].z<<" " ;
        file<<poses[i].yaw<<','<<poses[i].pitch<<','<<poses[i].roll<<'\n';        
    }


    file.close();
}


int main()
{
    //init isam
    isam::Slam *slam=new isam::Slam();
    
    
    unsigned seed=0;
    std::default_random_engine generator (seed);    
    
    float s=0.01;
    //float sr=0.01;
    float sl=0.001;
    
    std::normal_distribution<float> landMarkDistr (0.0,sl);
    std::normal_distribution<float> poseDistr (0.0,s);
    std::normal_distribution<float> rotDistr (0.0,s);
    
    //add prior
    isam::Pose3d_Node *prev_pose_node=new isam::Pose3d_Node();
    slam->add_node(prev_pose_node);
    Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6, 6)*SMALL_COV;
    isam::Noise noise=isam::Covariance(cov);
    isam::Pose3d origin(0,0,0,0,0,0);
    isam::Pose3d_Factor* prior = new isam::Pose3d_Factor(prev_pose_node, origin, noise);
    slam->add_factor(prior);
    
    //add landmarks
    for(int i=0;i<NL;i++)
    {
        double x,y,z;
        x=0.3*i;
        y=0.5;
        z=0.1;        
        isam::Point3d_Node *landmark=new isam::Point3d_Node();
        landmarks.push_back(landmark);
        slam->add_node(landmark);
    }
    
    
    //for all vo nodes
    for(int i=1;i<NN;i++)
    {
        PoseType poseDelta,gt;
        
        //0.2 forward on x axes        
        poseDelta.x=0.2;
        poseDelta.y=0;
        poseDelta.z=0;
        poseDelta.yaw=0;
        poseDelta.pitch=0;
        poseDelta.roll=0;
        
        gt=poseDelta;
        gt.x=poseDelta.x+0.2*i;
                
        
#ifdef ADD_NOISE        
        poseDelta.x+=poseDistr(generator);
        poseDelta.y+=poseDistr(generator);
        poseDelta.z+=poseDistr(generator);
        
        poseDelta.x+=rotDistr(generator);
        poseDelta.y+=rotDistr(generator);
        poseDelta.z+=rotDistr(generator);
#endif  
        truePoses.push_back(gt);
        
        
        isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node(); 
        slam->add_node(new_pose_node);
        
        
        isam::Pose3d vo(poseDelta.x,poseDelta.y,poseDelta.z,
                        poseDelta.yaw,poseDelta.pitch,poseDelta.roll);
        
        Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6, 6)*s;
        isam::Noise noise=isam::Covariance(cov);
        isam::Pose3d_Pose3d_Factor* factor = new isam::Pose3d_Pose3d_Factor(prev_pose_node,new_pose_node, vo, noise);
        slam->add_factor(factor);
        prev_pose_node=new_pose_node;
        
        
        //add landmark for this node
        if(i%SKIP_CON==0)
        {
            for(int lid=0;lid<NL;lid++)
            {
                double z=0.1;
                double y=0.5;
                double x=0.3*lid-0.2*i;

                if(fabs(x<1) && x>-1)
                {
#ifdef ADD_NOISE_
                    x+=landMarkDistr(generator);
                    y+=landMarkDistr(generator);
                    z+=landMarkDistr(generator);            
#endif
                    isam::Point3d point(x,y,z);
                    Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6, 6)*sl;
                    isam::Noise noise = isam::Covariance(cov);
                    isam::Pose3d_Point3d_Factor* f=new isam::Pose3d_Point3d_Factor(new_pose_node,landmarks[lid],point,noise);
                    slam->add_factor(f);
                }
            }
        }
            
    }
    
    //optimize
    char buf[32];
    sprintf(buf,"f_%d_graph",NN);
    slam->save(buf);

    slam->batch_optimization();


    sprintf(buf,"f_/f_%d_graph_new",NN);
    slam->save(buf);

    //slam->print_stats();
    
    double error=slam->chi2();
    std::cerr<<"Error:"<<error<<std::endl;
    
    
    std::cout<<"=============================="<<std::endl;
    
    
    sprintf(buf,"f_%d_poses",NN);
    savePoses(buf,truePoses);
    
    return 0;
}
