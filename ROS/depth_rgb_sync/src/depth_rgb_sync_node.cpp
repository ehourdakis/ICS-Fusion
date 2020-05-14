#include <depth_rgb_sync/depth_rgb_sync_utils.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_rgb_sync_node");
    ros::NodeHandle nh;

    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }

    depth_rgb_sync_utils su(nh);

    return 0;
}
