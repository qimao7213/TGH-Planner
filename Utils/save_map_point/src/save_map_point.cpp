#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>

bool has_writen = false;
std::string OUTPUT_FILE;
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if(has_writen) return;
    // 打开文件以写入被占据的栅格点坐标
    std::ofstream file(OUTPUT_FILE);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file for writing at \"%s\"", OUTPUT_FILE.c_str());
        return;
    }

    // 获取地图的分辨率
    float resolution = msg->info.resolution;

    // 遍历 costmap 数据
    int width = msg->info.width;
    int height = msg->info.height;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;      
           if(msg->data[index] == 0) { // 0 表示空闲
                file << x << " " << y << " 0"  << std::endl;
            }
            else if(msg->data[index] == 125) { // 125 表示未知
                file << x << " " << y << " 125"  << std::endl;
            } else  {  // 255 表示被占据
                // 记录到文件
                file << x << " " << y << " 255"  << std::endl;
                std::cout << static_cast<int>(msg->data[index]) << std::endl;
            }
        }
    }
    std::cout << std::endl;
    file.close();
    ROS_INFO("Occupied cells have been written!");
    has_writen = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_subscriber");
    ros::NodeHandle nh("~");
    nh.param<std::string>("output_file", OUTPUT_FILE, "xx/map_points.txt");
    // 订阅 costmap2d 话题
    ros::Subscriber costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/sdf_map/occupancy_2D", 1, costmapCallback);

    ros::spin();
    return 0;
}