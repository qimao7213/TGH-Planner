#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <chrono>
#include <random>

// CGAL 类型定义
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;

class Utils {
public:
    // OpenCV 的 cv::Point 转换为 CGAL 的 Point_2
    static Point_2 toCGALPoint(const cv::Point& pt) {
        return Point_2(pt.x, pt.y);
    }

    // CGAL 的 Point_2 转换为 OpenCV 的 cv::Point
    static cv::Point toOpenCVPoint(const Point_2& pt) {
        return cv::Point(static_cast<int>(CGAL::to_double(pt.x())),
                         static_cast<int>(CGAL::to_double(pt.y())));
    }

    // 生成随机颜色的 cv::Scalar
    static cv::Scalar randomColor() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<int> dis(0, 255);

        return cv::Scalar(dis(gen), dis(gen), dis(gen));
    }

    // 简单的计时器类
    class Timer {
    public:
        Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

        // 重置计时器
        void reset() {
            start_time_ = std::chrono::high_resolution_clock::now();
        }

        // 获取从启动或上次重置以来的时间（毫秒）
        double elapsedMilliseconds() const {
            auto end_time = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
        }

        // 获取从启动或上次重置以来的时间（秒）
        double elapsedSeconds() const {
            auto end_time = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double>(end_time - start_time_).count();
        }

    private:
        std::chrono::high_resolution_clock::time_point start_time_;
    };
};

#endif // UTILS_H