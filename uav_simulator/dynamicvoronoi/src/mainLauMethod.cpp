#include "dvr/voronoi_layer.h"

using namespace DynaVoro;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frame_map_node");
    ros::NodeHandle nh;

    VoronoiLayer voronoilayer(nh);

    ros::spin();

    return 0;
}