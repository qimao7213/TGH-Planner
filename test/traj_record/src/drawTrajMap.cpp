#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// 参数设置
const std::string src_path = "/home/bhrqhb/catkin_ws_fast_planner_sim/src/Fast-Planner/test/traj_record/experiment/bugtrap/";
const std::string world_image_path = src_path + "map.png"; // 输入地图图像路径
const std::string output_image_path = src_path + "trajectory_on_map2.png"; // 输出图像路径

const double resolution = 0.02; // 分辨率: 0.02m/pixel
// zigzag (-30， -32) min_speed = 0.5, max_speed = 2.0
// icra0  (-27， -32) min_speed = 0.5, max_speed = 2.3
// bugtrap (-30, -25) min_speed = 0.5, max_speed = 2.0;
const double origin_x = -30, origin_y = -25; // 世界坐标系原点
const double min_speed = 0.5, max_speed = 2.0; // 速度范围
const int min_l_value = 240;  // 最低 V 值：低速对应 75
const int max_l_value = 130; // 最高 V 值：高速对应 130   
const int line_width = 10;
// 世界坐标转像素坐标
cv::Point worldToPixel(double world_x, double world_y, double resolution, double origin_x, double origin_y) {
    int pixel_x = static_cast<int>((world_x - origin_x) / resolution);
    int pixel_y = static_cast<int>((world_y - origin_y) / resolution);
    return cv::Point(pixel_x, pixel_y);
}

// 速度到线宽映射
int speedToLineWidth(double speed, int min_linewidth, int max_linewidth) {
    speed = std::min(std::max(speed, min_speed), max_speed);
    return static_cast<int>(min_linewidth + (speed - min_speed) * (max_linewidth - min_linewidth) / (max_speed - min_speed));
}

// 速度到颜色映射 - 黄色到紫色
cv::Scalar speedToPurpleColor(double speed, int h = 0) {
    speed = std::min(std::max(speed, min_speed), max_speed);

    int interp_value = static_cast<int>(255 * (speed - min_speed) / (max_speed - min_speed));

    // 黄色到紫色：黄色 (0, 255, 255) -> 紫色 (255, 0, 255)
    int blue = interp_value;              // 蓝色从 0 增加到 255
    int green = 255 - interp_value;       // 绿色从 255 减少到 0
    return cv::Scalar(blue, green, 255);  // BGR 格式
}

// 速度到颜色映射 - 黄色到蓝色
cv::Scalar speedToBlueColor(double speed, int h = 0) {
    speed = std::min(std::max(speed, min_speed), max_speed);
    int gray_value = static_cast<int>(255 * (speed - min_speed) / (max_speed - min_speed));
    return cv::Scalar(gray_value, 255 - gray_value, 255 - gray_value); // BGR 格式
}

// 速度到颜色映射 - 黄色到绿色
cv::Scalar speedToGreenColor(double speed, int h = 0) {
    speed = std::min(std::max(speed, min_speed), max_speed);
    int red_value = static_cast<int>(255 * (1 - (speed - min_speed) / (max_speed - min_speed)));
    return cv::Scalar(0, 255, red_value); // BGR 格式
}

// 速度到颜色映射 - 黄色到青色
cv::Scalar speedToCyanColor(double speed, int h = 0) {
    speed = std::min(std::max(speed, min_speed), max_speed);

    int interp_value = static_cast<int>(255 * (speed - min_speed) / (max_speed - min_speed));

    // 黄色到青色：黄色 (0, 255, 255) -> 青色 (255, 255, 0)
    int blue = 255 - interp_value;        // 蓝色从 255 减少到 0
    int green = 255;                      // 绿色保持为 255
    return cv::Scalar(blue, green, interp_value);  // BGR 格式
}

// 速度到颜色映射 - 黄色到红色
cv::Scalar speedToRedColor(double speed, int h = 0) {
    speed = std::min(std::max(speed, min_speed), max_speed);
    int gray_value = static_cast<int>(255 * (speed - min_speed) / (max_speed - min_speed));
    return cv::Scalar(0, 255 - gray_value, 255); // BGR 格式
}

cv::Scalar speedToColorHSL(double speed, int h) {
    speed = std::min(std::max(speed, min_speed), max_speed);  // 限制速度范围
    const int s = 255;  // S 值：最大饱和度
    int use_max_l_value;
    use_max_l_value = (h == 85 ? max_l_value : max_l_value);
    // 根据速度映射亮度 (V) 的值
    int v = static_cast<int>(min_l_value + (speed - min_speed) * (double)(use_max_l_value - min_l_value) / (max_speed - min_speed));

    // 创建一个 1x1 的图像，使用指定的 H、S、V 值
    cv::Mat img_tmp(cv::Size(1, 1), CV_8UC3, cv::Scalar(h, v, s));

    // 转换颜色为 BGR 格式
    cv::Mat img_tmp2;
    cv::cvtColor(img_tmp, img_tmp2, cv::COLOR_HLS2BGR_FULL);

    // 获取 BGR 格式的颜色值
    cv::Vec3b color = img_tmp2.at<cv::Vec3b>(0, 0);
    return color;  // 返回 BGR 格式的颜色
}

// 封装函数 - 读取轨迹文件
void readTrajectory(const std::string& file_path, std::vector<cv::Point>& points, std::vector<double>& speeds) {
    std::ifstream trajectory_file(file_path);
    if (!trajectory_file.is_open()) {
        std::cerr << "Error: Cannot open trajectory file: " << file_path << std::endl;
        exit(-1);
    }

    double x, y, vel;
    while (trajectory_file >> x >> y >> vel) {
        points.push_back(worldToPixel(x, y, resolution, origin_x, origin_y));
        speeds.push_back(vel);
    }
    trajectory_file.close();
    std::cout << "Read " << points.size() << " points from " << file_path << std::endl;
}

// 封装函数 - 绘制轨迹
void drawTrajectoryOnMap(cv::Mat& map, const std::vector<cv::Point>& points, const std::vector<double>& speeds, 
                         cv::Scalar (*colorMapping)(double, int), int colorLValue, int line_width) {
    for (size_t i = 1; i < points.size(); ++i) {
        cv::Point prev_point = points[i - 1];
        cv::Point current_point = points[i];

        auto color = colorMapping(speeds[i - 1], colorLValue);
        color[3] = 100;
        // 检查像素坐标是否在图像范围内
        if (prev_point.x >= 0 && prev_point.x < map.cols && prev_point.y >= 0 && prev_point.y < map.rows &&
            current_point.x >= 0 && current_point.x < map.cols && current_point.y >= 0 && current_point.y < map.rows) {
            // 绘制线条
            cv::line(map, prev_point, current_point, color, line_width, cv::LINE_8, 0);
        }
    }
}

int main() {
    // 加载世界地图图像
    cv::Mat world_img = cv::imread(world_image_path, -1);
    if (world_img.empty()) {
        std::cerr << "Error: Cannot load world image: " << world_image_path << std::endl;
        return -1;
    }
    std::cout << "World image loaded successfully." << std::endl;
    // int sum = cv::countNonZero(world_img);
    // 转换为 BGR 图像
    cv::Mat world_img_rgb;
    cv::cvtColor(world_img, world_img_rgb, cv::COLOR_GRAY2BGR);

    // 定义轨迹文件路径
    // icra0: FASTER8, TEB2,TGH6
    // zigzag7: FASTER2, TEB1, TGH6
    // zigzag8: FASTER3, TEB1, TGH5
    // bugtrap: FASTER1, TEB1, TGH2
    std::vector<std::string> trajectory_files = {
        src_path + "FASTER1_vel.txt",        
        src_path + "TEB1_vel.txt"  ,      
        src_path + "TGH2_vel.txt"
    };

    // 定义颜色映射函数列表（每条轨迹使用不同颜色映射）
    std::vector<cv::Scalar (*)(double, int)> colorMappings = {
        speedToColorHSL,
        speedToColorHSL,
        speedToColorHSL
    };
    // 42:黄色，127:蓝色，0:红色
    // 85:绿色，170：蓝色, 0:红色
    // 
    std::vector<int> colorLValue = {170, 85, 0};

    // 绘制所有轨迹
    for (size_t i = 0; i < trajectory_files.size(); ++i) {
        std::vector<cv::Point> trajectory_points;
        std::vector<double> trajectory_vels;

        // 读取轨迹文件
        readTrajectory(trajectory_files[i], trajectory_points, trajectory_vels);

        // 绘制轨迹到地图
        drawTrajectoryOnMap(world_img_rgb, trajectory_points, trajectory_vels, 
                            colorMappings[i], colorLValue[i], line_width); // 固定线宽为5
    }

    // 保存和显示结果
    cv::imwrite(output_image_path, world_img_rgb);
    std::cout << "Output image saved to: " << output_image_path << std::endl;

    // cv::imshow("Trajectory on Map", world_img_rgb);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    return 0;
}
