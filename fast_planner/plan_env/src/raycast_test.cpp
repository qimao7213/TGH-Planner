#include <plan_env/raycast.h>
#include <iostream>
#include <vector>
#include <fstream>


void savePointsToFile(const std::vector<Eigen::Vector3d>& points, const std::string& filename) {
    // 创建输出文件流对象
    std::ofstream outFile(filename);
    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    // 将vector中的每个元素写入文件
    int iter = 0;
    for (const Eigen::Vector3d& pt : points) {
        outFile << pt(0) << " " << pt(1) << "\n";
        iter ++;
    }
    // 关闭文件
    outFile.close();
    // 检查文件是否成功关闭
    if (!outFile) {
        std::cerr << "Error closing file: " << filename << std::endl;
    }
}

int main(int argc, char* argv[])
{
    double skip_scale = 4.0;
    // double skip_scale = std::stod(argv[1]);
    Eigen::Vector3d ray_pt;
    Eigen::Vector3d pt_id;
    double resolution = 0.1;
    RayCaster raycaseter;
    Eigen::Vector3d start_pt;
    Eigen::Vector3d end_pt;
    start_pt << -2.54989,  -4.7453, -0.499288;
    end_pt << 2,  2,  2;
    Eigen::Vector3d dir = end_pt - start_pt;
    int step_num;
    if(abs(dir.x()) < resolution || abs(dir.y()) < resolution)
    {
        step_num = round(skip_scale); 
    }
    else
    {
        double tan_yaw = dir.y() / dir.x();
        double step_x = (sqrt(skip_scale * skip_scale / (1 + tan_yaw * tan_yaw)));
        double step_y = (step_x * tan_yaw);
        step_num = round(step_x + step_y);        
    }

    std::vector<Eigen::Vector3d> step_pts;
    int iter = 0;    
    // step_pts.emplace_back(start_pt); 
    raycaseter.setInput(start_pt / resolution, end_pt / resolution);
    // 注意，step到达终点的时候就返回false，while不会执行到达终点的那一个
    while (raycaseter.step(ray_pt, 1)) {
        // if(iter == 0 || (iter % step_num) != 0) 
        // {
        //     ++iter;
        //     continue;
        // }
        pt_id(0) = ray_pt(0) * 1.0 * resolution;
        pt_id(1) = ray_pt(1) * 1.0 * resolution;
        pt_id(2) = ray_pt(2) * 1.0 * resolution;
        step_pts.emplace_back(pt_id);    
        int debug = 0;
    }
    step_pts.emplace_back(ray_pt * resolution); 
    savePointsToFile(step_pts, "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_env/src/ray_pts_1.txt");


    iter = 0;   
    step_pts.resize(0); 
    step_pts.emplace_back(start_pt); 
    raycaseter.setInput(start_pt / resolution, end_pt / resolution);
    // 注意，step到达终点的时候就返回false，while不会执行到达终点的那一个
    while (raycaseter.step(ray_pt, 1)) {
        if(iter == 0 || (iter % step_num) != 0) 
        {
            ++iter;
            continue;
        }
        ++iter;
        pt_id(0) = ray_pt(0) * 1.0 * resolution;
        pt_id(1) = ray_pt(1) * 1.0 * resolution;
        pt_id(2) = ray_pt(2) * 1.0 * resolution;
        step_pts.emplace_back(pt_id);    
        int debug = 0;
    }
    step_pts.emplace_back(end_pt); 
    savePointsToFile(step_pts, "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_env/src/ray_pts.txt");

    // iter = 0;   
    // step_pts.resize(0); 
    // step_pts.emplace_back(start_pt); 
    // raycaseter.setInput(start_pt / resolution, end_pt / resolution);
    // // 注意，step到达终点的时候就返回false，while不会执行到达终点的那一个
    // while (raycaseter.step(ray_pt, 1)) {
    //     if(iter == 0 || (iter % 3) != 0) 
    //     {
    //         ++iter;
    //         continue;
    //     }
    //     ++iter;
    //     pt_id(0) = ray_pt(0) * 1.0 * resolution;
    //     pt_id(1) = ray_pt(1) * 1.0 * resolution;
    //     pt_id(2) = ray_pt(2) * 1.0 * resolution;
    //     step_pts.emplace_back(pt_id);    
    //     int debug = 0;
    // }
    // step_pts.emplace_back(end_pt); 
    // savePointsToFile(step_pts, "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_env/src/ray_pts_scale2.txt");

    // iter = 0;   
    // step_pts.resize(0); 
    // step_pts.emplace_back(start_pt); 
    // raycaseter.setInput(start_pt / resolution, end_pt / resolution);
    // // 注意，step到达终点的时候就返回false，while不会执行到达终点的那一个
    // while (raycaseter.step(ray_pt, 1)) {
    //     if(iter == 0 || (iter % 6) != 0) 
    //     {
    //         ++iter;
    //         continue;
    //     }
    //     ++iter;
    //     pt_id(0) = ray_pt(0) * 1.0 * resolution;
    //     pt_id(1) = ray_pt(1) * 1.0 * resolution;
    //     pt_id(2) = ray_pt(2) * 1.0 * resolution;
    //     step_pts.emplace_back(pt_id);    
    //     int debug = 0;
    // }
    // step_pts.emplace_back(end_pt); 
    // savePointsToFile(step_pts, "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/fast_planner/plan_env/src/ray_pts_scale3.txt");


    return 0;
}




