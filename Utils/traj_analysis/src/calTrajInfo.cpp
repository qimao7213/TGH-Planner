#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <cmath>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::string fig_name;
void plotData(const std::vector<Eigen::Vector3d> &data, const std::string& title)
{
    plt::figure(); // 每次调用时创建新图形
    // 创建四个子图
    // plt::subplot(2, 2, 1);
    std::vector<double> xValues, yValues;
    for (const auto& v : data) {
        xValues.push_back(v[0]);
    }
    // plt::plot(xValues, {{"label", "X"}}); // 绘制X维度的数据

    // plt::subplot(2, 2, 2);
    yValues.clear();
    for (const auto& v : data) {
        yValues.push_back(v[1]);
    }
    // plt::plot(yValues, {{"label", "Y"}}); // 绘制Y维度的数据

    // // plt::subplot(2, 2, 3);
    // std::vector<double> zValues;
    // for (const auto& v : data) {
    //     zValues.push_back(v[2]);
    // }
    // plt::plot(zValues, {{"label", "Z"}}); // 绘制Z维度的数据

    // plt::subplot(2, 2, 4);
    std::vector<double> norms;
    for (const auto& v : data) {
        norms.push_back(v.norm()); // 计算并保存每个向量的范数
    }
    plt::plot(norms, {{"label", "Norm"}}); // 绘制每个向量的范数
    plt::title(title);
    plt::legend();
    // plt::figure_size(1920 / 100, 1080 / 100);
    plt::save(fig_name + ".png", 300);
}
double quaternion_to_yaw(const Eigen::Quaterniond& q) 
{
    // 提取四元数的分量
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    // 计算偏航角
    double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0* (y * y + z * z));
    return yaw; // 返回偏航角，单位为弧度
}

void saveVelProfile(const std::vector<Eigen::Vector3d> &pos_data, 
                    const std::vector<Eigen::Vector3d> &vel_data, 
                    const std::string& file_path)
{
    // 使用ofstream来写入数据
    std::ofstream file(file_path);
    
    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return; // 提前返回，避免写入错误
    }

    // 写入数据
    for (size_t i = 0; i < vel_data.size(); ++i)
    {
        file << pos_data[i].x() << " " << pos_data[i].y() << " " <<  vel_data[i].norm() << std::endl;
    }

    // 关闭文件
    file.close();
}

struct Pose {
    double timestamp;
    Eigen::Vector3d position; // Position
    Eigen::Quaterniond orientation; // Orientation (quaternion)
    double yaw;
};
void check_and_remove_duplicates(std::vector<Pose>& poses) {
    std::set<double> seen_timestamps;    // 用于存储已经出现过的时间戳
    std::vector<Pose> unique_poses;      // 存储去重后的 Pose 数据

    for (const auto& pose : poses) {
        if (seen_timestamps.find(pose.timestamp) == seen_timestamps.end()) {
            // 如果时间戳未出现过，则保留此 Pose
            unique_poses.push_back(pose);
            seen_timestamps.insert(pose.timestamp);
        } else {
            std::cerr << "Duplicate timestamp found: " << pose.timestamp << std::endl;
        }
    }

    poses = std::move(unique_poses); // 更新原始 poses 数据
}



std::vector<Pose> load_tum_trajectory(const std::string& file_path) {
    std::vector<Pose> poses;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Pose pose;
        double qx, qy, qz, qw;

        if (!(iss >> pose.timestamp >> pose.position[0] >> pose.position[1] >> pose.position[2] >> qx >> qy >> qz >> qw)) {
            continue; // Skip invalid lines
        }
        pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz); // 注意 Eigen 的 Quaternion 顺序
        pose.yaw = quaternion_to_yaw(pose.orientation);
        poses.push_back(pose);
    }
    return poses;
}


double calculate_duration(const std::vector<Pose>& poses) {
    return poses.back().timestamp - poses.front().timestamp;
}

double calculate_path_length(const std::vector<Pose>& poses) {
    double path_length = 0.0;
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        path_length += (poses[i + 1].position - poses[i].position).norm();
    }
    return path_length;
}

double calculate_average_speed(double path_length, double duration) {
    return path_length / duration;
}

std::vector<Eigen::Vector3d> calculate_velocities(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> velocities;
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp;
        Eigen::Vector3d velocity = (poses[i + 1].position - poses[i].position) / dt;
        velocities.push_back(velocity);
    }
    return velocities;
}

std::vector<Eigen::Vector3d> calculate_velocities2(const std::vector<Pose>& poses, std::vector<Eigen::Vector3d>& sample_pos, int interval) {
    std::vector<Eigen::Vector3d> velocities;
    for (size_t i = 0; i + interval < poses.size(); i += interval) {
        double dt = poses[i + interval].timestamp - poses[i].timestamp;
        if (dt > 0) {
            Eigen::Vector3d velocity = (poses[i + interval].position - poses[i].position) / dt;
            velocities.push_back(velocity);
            sample_pos.push_back((poses[i].position));
        }
    }
    return velocities;
}

std::vector<Eigen::Vector3d> calculate_velocities3(const std::vector<Eigen::Vector3d>& velocities, int window_size) {
    std::vector<Eigen::Vector3d> smoothed_velocities;
    int half_window = window_size / 2;

    for (size_t i = 0; i < velocities.size(); ++i) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;

        // 计算窗口内的平均值
        for (int j = -half_window; j <= half_window; ++j) {
            if (i + j >= 0 && i + j < velocities.size()) {
                sum += velocities[i + j];
                count++;
            }
        }

        smoothed_velocities.push_back(sum / count);
    }
    return smoothed_velocities;
}

std::vector<Eigen::Vector3d> calculate_accelerations(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> accelerations;
    for (size_t i = 1; i < poses.size() - 2; ++i) {
        double dt = poses[i + 1].timestamp - poses[i + 0].timestamp; 
        Eigen::Vector3d acceleration = (poses[i + 1].position + poses[i - 1].position - 2 * poses[i].position) / (dt * dt);
        accelerations.push_back(acceleration);
    }
    return accelerations;
}

std::vector<Eigen::Vector3d> calculate_jerks(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> jerks;
    for (size_t i = 2; i < poses.size() - 3; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp; // t(i+1) - t(i)
        Eigen::Vector3d jerk = (poses[i + 2].position - 3 * poses[i + 1].position +
                                3 * poses[i + 0].position - poses[i - 1].position) / (dt * dt * dt);
        jerks.push_back(jerk);
    }
    return jerks;
}

std::vector<double> calculate_angular_velocities2(const std::vector<Pose>& poses) {
    std::vector<double> angular_velocities;
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp;
        double angular_velocity = (poses[i + 1].yaw - poses[i].yaw) / dt;
        angular_velocities.push_back(angular_velocity);
    }
    return angular_velocities;
}

std::vector<double> calculate_angular_accelerations2(const std::vector<Pose>& poses) {
    std::vector<double> accelerations;
    for (size_t i = 1; i < poses.size() - 2; ++i) {
        double dt = poses[i + 1].timestamp - poses[i + 0].timestamp; 
        double acceleration = (poses[i + 1].yaw + poses[i - 1].yaw - 2 * poses[i].yaw) / (dt * dt);
        accelerations.push_back(acceleration);
    }
    return accelerations;
}

std::vector<double> calculate_angular_jerks2(const std::vector<Pose>& poses) {
    std::vector<double> jerks;
    for (size_t i = 2; i < poses.size() - 3; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp; // t(i+1) - t(i)
        double jerk = (poses[i + 2].yaw - 3 * poses[i + 1].yaw +
                                3 * poses[i + 0].yaw - poses[i - 1].yaw) / (dt * dt * dt);
        jerks.push_back(jerk);
    }
    return jerks;
}


std::vector<Eigen::Vector3d> calculate_angular_velocities(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> angular_velocities;
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp;
        const auto& q1 = poses[i + 0].orientation.normalized();
        const auto& q2 = poses[i + 1].orientation.normalized();

        Eigen::Quaterniond delta_q = (q2 * q1.conjugate()).normalized(); // 计算四元数的差
        Eigen::Vector3d angular_velocity = 2 * Eigen::Vector3d(delta_q.x(), delta_q.y(), delta_q.z()) / dt;
        angular_velocities.push_back(angular_velocity);
    }
    return angular_velocities;
}

std::vector<Eigen::Vector3d> calculate_angular_accelerations(const std::vector<Pose>& poses, const std::vector<Eigen::Vector3d> angular_velocities) {
    std::vector<Eigen::Vector3d> angular_accelerations;
    for (size_t i = 0; i < angular_velocities.size() - 1; ++i) {
        double dt = poses[i + 1].timestamp - poses[i + 0].timestamp; 
        Eigen::Vector3d angular_acceleration = (angular_velocities[i + 1] - angular_velocities[i + 0])/ (dt);
        angular_accelerations.push_back(angular_acceleration);
    }
    return angular_accelerations;
}

std::vector<Eigen::Vector3d> calculate_angular_jerks(const std::vector<Pose>& poses, const std::vector<Eigen::Vector3d>& angular_accelerations) {
    std::vector<Eigen::Vector3d> angular_jerks;
    for (size_t i = 0; i < angular_accelerations.size() + 1; ++i) {
        double dt = poses[i + 1].timestamp - poses[i].timestamp; // t(i+1) - t(i)
        Eigen::Vector3d angular_jerk = (angular_accelerations[i] - angular_accelerations[i - 1]) / dt;
        angular_jerks.push_back(angular_jerk);
    }
    return angular_jerks;
}

double calculate_average(const std::vector<double>& vec) {
    double sum = 0;
    for(const auto & value : vec) sum += std::abs(value);
    return sum / vec.size(); 
}

double calculate_average(const std::vector<Eigen::Vector3d>& vec) {
    double sum = 0;
    for(const auto & value : vec) 
    {   
        sum += value.norm();
    }
    return sum / vec.size(); 
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <trajectory_file.txt>" << std::endl;
        return 1;
    }

    std::string file_path = argv[1];
    auto poses = load_tum_trajectory(file_path);
    check_and_remove_duplicates(poses);
    if (poses.empty()) {
        std::cerr << "Failed to load trajectory data." << std::endl;
        return 1;
    }

    double duration = calculate_duration(poses);
    double path_length = calculate_path_length(poses);
    double average_speed = calculate_average_speed(path_length, duration);

    auto velocities = calculate_velocities(poses);
    auto accelerations = calculate_accelerations(poses);
    auto jerks = calculate_jerks(poses);

    auto angular_velocities = calculate_angular_velocities(poses);
    auto angular_accelerations = calculate_angular_accelerations(poses, angular_velocities);
    auto angular_jerks = calculate_angular_jerks(poses, angular_accelerations);

    auto angular_velocities2 = calculate_angular_velocities2(poses);
    auto angular_accelerations2 = calculate_angular_accelerations2(poses);
    auto angular_jerks2 = calculate_angular_jerks2(poses);


    std::cout << "Duration (s): " << duration << std::endl;
    std::cout << "Path Length (m): " << path_length << std::endl;

    std::cout << "---" << std::endl;
    std::cout << "Average Speed (m/s): " << average_speed << std::endl;
    std::cout << "Average Velocity (m/s): " << calculate_average(velocities) << std::endl;
    std::cout << "Average Acceleration (m/s²): " << calculate_average(accelerations) << std::endl;
    std::cout << "Average Jerk (m/s³): " << calculate_average(jerks) << std::endl;

    std::cout << "---" << std::endl;
    std::cout << "Average Angular Velocity (rad/s): " << calculate_average(angular_velocities) << std::endl;
    std::cout << "Average Angular Acceleration (rad/s²): " << calculate_average(angular_accelerations) << std::endl;
    std::cout << "Average Angular Jerk (rad/s³): " << calculate_average(angular_jerks) << std::endl;

    std::cout << "---" << std::endl;
    std::cout << "Average Angular Velocity 222 (rad/s): " << calculate_average(angular_velocities2) << std::endl;
    std::cout << "Average Angular Acceleration 222 (rad/s²): " << calculate_average(angular_accelerations2) << std::endl;
    std::cout << "Average Angular Jerk 222(rad/s³): " << calculate_average(angular_jerks2) << std::endl;


    std::vector<Eigen::Vector3d> sample_pos;
    int interval = 10; // 每5个点计算一次速度
    std::vector<Eigen::Vector3d> velocities2 = calculate_velocities2(poses, sample_pos, interval);
    std::vector<Eigen::Vector3d> velocities3 = calculate_velocities3(velocities2, 7); // 平滑窗口大小为5
    size_t dot_pos = file_path.rfind('.');
    std::string new_file = file_path.substr(0, dot_pos) + "_vel" + file_path.substr(dot_pos);
    
    fig_name = file_path.substr(0, dot_pos) + "_vel";
    // plotData(velocities, "velocities");  
    // plotData(velocities2, "velocities");  
    plotData(velocities3, "velocities");  


    std::cout << "Data Size: " << sample_pos.size() << ", " << velocities3.size() << std::endl;

    saveVelProfile(sample_pos, velocities3, new_file);    
    plt::show();
    return 0;
}

