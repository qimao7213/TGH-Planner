#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

/*Description:
    Draw trajectories with direction arrows on the world map.
    The trajectory file path is "trajectory_files".
*/

// 参数设置
// 这个路径要手动修改，因为不想接入到ros里面
const std::string src_path = "xx/perception/";
const std::string world_image_path = src_path + "map.png"; // 输入地图图像路径
const std::string output_image_path = src_path + "traj_on_map.png"; // 输出图像路径

const double resolution = 0.005; // 分辨率: 0.02m/pixel
const double arrowLength = 1.0; // m
const double angleFOV = 80;     // 度
// zigzag (-30， -32) min_speed = 0.5, max_speed = 2.0
// icra0  (-27， -32) min_speed = 0.5, max_speed = 2.3
// bugtrap (-30, -25) min_speed = 0.5, max_speed = 2.0;
// perception (-7, -7)
const double origin_x = -4, origin_y = -4; // 世界坐标系原点
const double min_speed = 0.5, max_speed = 2.0; // 速度范围
const int min_l_value = 240;  // 最低 V 值：低速对应 75
const int max_l_value = 130; // 最高 V 值：高速对应 130   
const int line_width = 5;
// 世界坐标转像素坐标
cv::Point worldToPixel(double world_x, double world_y, double resolution, double origin_x, double origin_y) {
    int pixel_x = static_cast<int>((world_x - origin_x) / resolution);
    int pixel_y = static_cast<int>((world_y - origin_y) / resolution);
    return cv::Point(pixel_x, pixel_y);
}

std::vector<cv::Scalar> color_set;


// 封装函数 - 读取轨迹文件
void readTrajectory(const std::string& file_path, std::vector<cv::Point>& points) {
    std::ifstream trajectory_file(file_path);
    if (!trajectory_file.is_open()) {
        std::cerr << "Error: Cannot open trajectory file: " << file_path << std::endl;
        exit(-1);
    }
    double x, y, t;
    std::string line;
    
    while (std::getline(trajectory_file, line)) {
        std::istringstream iss(line);
        double temp;
        // 读取前三个数据
        if (iss >> t >> x >> y) {
            points.push_back(worldToPixel(x, y, resolution, origin_x, origin_y));
        } else {
            // 处理读取错误或数据不足的情况
            std::cerr << "Error reading data from line: " << line << std::endl;
        }
        // 忽略该行剩余的数据
        while (iss >> temp) {
            // 仅忽略剩余数据，不做处理
        }
    }
    trajectory_file.close();
    std::cout << "Read " << points.size() << " points." << std::endl;
}

cv::Point2f normalizeDirection(cv::Point2f dir) 
{
    float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    return (length > 1e-6) ? cv::Point2f(dir.x / length, dir.y / length) : cv::Point2f(0, 0); 
}
cv::Point2f rotateVector(cv::Point2f vec, float angle_deg) 
{
    float angle_rad = angle_deg * CV_PI / 180.0;  // 转换为弧度
    float cosA = std::cos(angle_rad);
    float sinA = std::sin(angle_rad);
    return cv::Point2f(vec.x * cosA - vec.y * sinA, vec.x * sinA + vec.y * cosA); 
}


// 绘制箭头（手动指定箭头方向）
void drawArrowOnMap(cv::Mat& map, cv::Point position, cv::Point2f direction, int arrow_length, int thickness) {
    // 计算箭头终点
    cv::Point2f dir_normlized = normalizeDirection(direction);
    cv::Point2f dir_rotated = rotateVector(dir_normlized, 0.5 * angleFOV);
    cv::Point arrow_tip = position + cv::Point(dir_rotated.x * arrow_length, dir_rotated.y * arrow_length);

    // 确保箭头不超出边界
    if (arrow_tip.x >= 0 && arrow_tip.x < map.cols && arrow_tip.y >= 0 && arrow_tip.y < map.rows) {
        cv::arrowedLine(map, position, arrow_tip, cv::Scalar(255, 0, 0), thickness, 16, 0, 0.3);
    }
}

// 封装函数 - 绘制轨迹
void drawTrajectoryWithDirOnMap(cv::Mat& map, const std::vector<cv::Point>& points, 
                                std::vector<cv::Point2f>& trajdirs, int line_width, int traj_id) {
    for (size_t i = 1; i < points.size(); ++i) {
        cv::Point prev_point = points[i - 1];
        cv::Point current_point = points[i];

        auto color = color_set[traj_id];
        // 检查像素坐标是否在图像范围内
        if (prev_point.x >= 0 && prev_point.x < map.cols && prev_point.y >= 0 && prev_point.y < map.rows &&
            current_point.x >= 0 && current_point.x < map.cols && current_point.y >= 0 && current_point.y < map.rows) {
            // 绘制线条
            cv::line(map, prev_point, current_point, color, line_width, cv::LINE_8, 0);
        }
        // // 每 arrow_interval（例如 10）个点绘制一个箭头
        // if (i % 10 == 0 && i < trajdirs.size()) {
        //     drawArrowOnMap(map, points[i], trajdirs[i], arrowLength/resolution, 2);  // 箭头长度 15
        // }

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

    color_set.push_back(cv::Scalar(255  , 103, 103));//浅蓝

    color_set.push_back(cv::Scalar(99, 255 , 79));//浅绿
    color_set.push_back(cv::Scalar(0, 213, 27)); //深绿
    color_set.push_back(cv::Scalar(50,  113,  234)); //橙色
    color_set.push_back(cv::Scalar(21, 21, 255 )); //金色    
    color_set.push_back(cv::Scalar(77 , 190, 238));//深红
    color_set.push_back(cv::Scalar(162, 20 , 47)); //浅蓝
    color_set.push_back(cv::Scalar(86, 180, 233)); //棕色
    color_set.push_back(cv::Scalar(166, 86, 40));
    // color_set.push_back(cv::Scalar(0, 114, 189));
    // color_set.push_back(cv::Scalar(0, 114, 189));
    // color_set.push_back(cv::Scalar(0, 114, 189));
    // 定义轨迹文件路径
    // icra0: FASTER8, TEB2,TGH6
    // zigzag7: FASTER2, TEB1, TGH6
    // zigzag8: FASTER3, TEB1, TGH5
    // bugtrap: FASTER1, TEB1, TGH2
    // perception: TEB1, faster_agv1, faster_uav3, TGH-no2, TGH-per4
    std::vector<std::string> trajectory_files = {
        src_path + "TEB1.txt",        
        src_path + "faster_agv1.txt",      
        src_path + "faster_uav3.txt",
        src_path + "TGH-no2.txt",
        src_path + "TGH-per4.txt",
    };


    // 绘制所有轨迹
    for (size_t i = 0; i < trajectory_files.size(); ++i) {
        std::vector<cv::Point> trajectory_points;
        std::vector<double> trajectory_vels;
        std::vector<cv::Point2f> trajectory_dirs;
        // 读取轨迹文件
        readTrajectory(trajectory_files[i], trajectory_points);

        drawTrajectoryWithDirOnMap(world_img_rgb, trajectory_points, trajectory_dirs, line_width, i); // 固定线宽为5

    }

    // 保存和显示结果
    cv::imwrite(output_image_path, world_img_rgb);
    std::cout << "Output image saved to: " << output_image_path << std::endl;

    // cv::imshow("Trajectory on Map", world_img_rgb);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    return 0;
}
