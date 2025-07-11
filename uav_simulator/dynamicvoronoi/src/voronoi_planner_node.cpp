#include "dvr/VoronoiPlanner.h"
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_topo_rpm");
    ros::NodeHandle nh("~");

    VoronoiPlanner voronoi_planner;
    voronoi_planner.init(nh);
    ros::Rate rate(100);
    while (ros::ok()) {
        voronoi_planner.run();
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;

}
