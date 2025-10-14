#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <matplotlibcpp.h>

double dt_ = 0.1;
struct Pose {
    double timestamp;
    Eigen::Vector3d position; // Position
    Eigen::Quaterniond orientation; // Orientation (quaternion)
    double yaw;
};
namespace plt = matplotlibcpp;

void plotData(const std::vector<Eigen::Vector3d> &data, const std::string& title)
{
    plt::figure(); // 每次调用时创建新图形
    // 创建四个子图
    // plt::subplot(2, 2, 1);
    std::vector<double> xValues, yValues;
    for (const auto& v : data) {
        xValues.push_back(v[0]);
    }
    plt::plot(xValues, {{"label", "X"}}); // 绘制X维度的数据

    // plt::subplot(2, 2, 2);
    yValues.clear();
    for (const auto& v : data) {
        yValues.push_back(v[1]);
    }
    plt::plot(yValues, {{"label", "Y"}}); // 绘制Y维度的数据

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
}





std::vector<Pose> load_geo_path(const std::string& file_path) {
    std::vector<Pose> poses;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Pose pose;
        double qx, qy, qz, qw;

        if (!(iss >> pose.position[0] >> pose.position[1])) {
            continue; // Skip invalid lines
        }
        pose.position[2] = 0.0;
        poses.push_back(pose);
    }
    return poses;
}

std::vector<Eigen::Vector3d> calculate_velocities(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> velocities;
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        double dt = dt_;
        Eigen::Vector3d velocity = (poses[i + 1].position - poses[i].position) / dt;
        velocities.push_back(velocity);
    }
    return velocities;
}

std::vector<Eigen::Vector3d> calculate_accelerations(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> accelerations;
    for (size_t i = 1; i < poses.size() - 2; ++i) {
        double dt = dt_; 
        Eigen::Vector3d acceleration = (poses[i + 1].position + poses[i - 1].position - 2 * poses[i].position) / (dt * dt);
        accelerations.push_back(acceleration);
    }
    return accelerations;
}

std::vector<Eigen::Vector3d> calculate_jerks(const std::vector<Pose>& poses) {
    std::vector<Eigen::Vector3d> jerks;
    for (size_t i = 2; i < poses.size() - 3; ++i) {
        double dt = dt_; // t(i+1) - t(i)
        Eigen::Vector3d jerk = (poses[i + 2].position - 3 * poses[i + 1].position +
                                3 * poses[i + 0].position - poses[i - 1].position) / (dt * dt * dt);
        jerks.push_back(jerk);
    }
    return jerks;
}


double calculate_average(const std::vector<double>& vec) {
    double sum = 0;
    for(const auto & value : vec) sum += std::abs(value);
    return sum / vec.size(); 
}

double calculate_average(const std::vector<Eigen::Vector3d>& vec) {
    double sum = 0;
    for(const auto & value : vec) sum += value.norm();
    return sum / vec.size(); 
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_file.txt>" << std::endl;
        return 1;
    }

    std::string file_path = argv[1];
    auto poses = load_geo_path(file_path);
    
    if (poses.empty()) {
        std::cerr << "Failed to load trajectory data." << std::endl;
        return 1;
    }


    auto velocities = calculate_velocities(poses);
    auto accelerations = calculate_accelerations(poses);
    auto jerks = calculate_jerks(poses);


    std::cout << "---" << std::endl;
    std::cout << "Average Velocity (m/s): " << calculate_average(velocities) << std::endl;
    std::cout << "Average Acceleration (m/s²): " << calculate_average(accelerations) << std::endl;
    std::cout << "Average Jerk (m/s³): " << calculate_average(jerks) << std::endl;
    plotData(velocities, "velocities");
    plotData(accelerations, "accelerations");
    plotData(jerks, "jerks");
    // 显示图形
    plt::show();

    return 0;
}