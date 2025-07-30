
#include "TestTopoRpm.h"
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
using namespace fast_planner;
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_topo_rpm");
    ros::NodeHandle nh("~");

    // cv::Mat img1, img2;
    // img1 = cv::imread("/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_manage/maps/map_maze2.png", -1);
    // img2 = cv::imread("/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_manage/maps/perception_map.png", -1);
    TestTopoRpm test_topo_rpm;
    test_topo_rpm.init(nh);

    int iter = 0; 
    ros::Rate rate(10);
    while (ros::ok()) {
        if (iter == 3) 
        {
            int debug = 0;
        }
        ros::spinOnce();
        test_topo_rpm.run();
        // std::cout << "iter:" << iter << std::endl;
        // 用来计算一段轨迹的esdf值。需要的时候就打开使用
        // if(iter == 20)
        // {
        //     for (int i = 0; i < 10; ++i)
        //     {
        //         std::string trajFilePah = "/home/bhrqhb/catkin_ws/planner/catkin_ws_TGH_Planner/src/TGH-Planner/test/traj_record/data/TGH_highhigh/traj" + to_string(i);
        //         test_topo_rpm.calTrajESDF(trajFilePah);   
        //         int debug = 0;             
        //     }

        // }
        iter++;
        
        rate.sleep();
    }
    test_topo_rpm.close_file();
    ros::shutdown();
    return 0;


}
