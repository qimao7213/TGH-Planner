#include <iostream>
#include <nlopt.hpp>
#include <cmath>


int iter_num_ = 0;

// 定义目标函数，NLopt 的优化算法会调用此函数
double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    // 如果需要梯度信息，可以计算并赋值到 grad，但这里不使用梯度
    grad.resize(1, 0);
    if (!grad.empty()) {
        grad[0] = 2 * (x[0] - 2); // 计算导数
    }
    
    std::cout << "iter: " << iter_num_ << ", x: " << x[0] << ", grad: " << grad[0] << ", cost: "
              << (x[0] - 2) * (x[0] - 2) << std::endl;

    iter_num_ ++;
    return (x[0] - 2) * (x[0] - 2); // 返回目标函数的值 (x - 2)^2
}

int main() {
    // 创建一个1维的优化器，使用LN_NELDERMEAD算法
    nlopt::opt opt(nlopt::LD_TNEWTON, 1);
    std::cout << "---" << std::endl;
    // 设置目标函数
    opt.set_min_objective(objective_function, nullptr);

    // 设置变量的上下界 (这里假设在[-10, 10]区间优化)
    opt.set_lower_bounds(-10.0);
    opt.set_upper_bounds(10.0);

    // 设置优化的停止条件
    opt.set_xtol_rel(1e-4); // 相对容差

    // 初始值
    std::vector<double> x(1, 0.0); // 从 x = 0 开始

    double minf; // 存储最小值
    nlopt::result result = opt.optimize(x, minf);

    if (result == nlopt::SUCCESS || result == nlopt::XTOL_REACHED) {
        std::cout << "找到最小值 x = " << x[0] << "，f(x) = " << minf << std::endl;
    } else {
        std::cout << "优化失败。" << std::endl;
    }

    return 0;
}

