
#include "TopoRpmPlan.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_topo_rpm");
    ros::NodeHandle nh("~");

    TestTopoRpm test_topo_rpm;
    ros::Duration(1.0).sleep(); // 等待ROS系统稳定
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
        //         std::string trajFilePah = "xx/src/TGH-Planner/test/traj_record/data/TGH_highhigh/traj" + to_string(i);
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
